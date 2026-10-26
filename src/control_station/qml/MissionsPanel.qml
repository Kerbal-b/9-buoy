import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtWebEngine
import QtWebChannel

Item {
    id: root
    property var backendObject
    property var backendState: ({})
    property var plan: backendState.missionDraft || ({})
    property int missionRevision: backendState.missionRevision === undefined ? -1 : backendState.missionRevision
    property bool missionReady: false
    property string tool: "select"
    property string selectedKind: ""
    property int selectedIndex: -1
    property string mapHint: "Drag to pan, scroll or pinch to zoom"
    property string mapStyle: "satellite"
    property string editMode: "points"
    property bool webMapReady: false
    property bool planPanelOpen: false
    readonly property real startLatitude: 37.53188
    readonly property real startLongitude: -122.24802
    readonly property real startZoom: 14
    property string lastFittedPlanId: ""
    readonly property color ink: "#e6ecf2"
    readonly property color muted: "#9eb5c7"

    function runMap(script) {
        if (webMapReady) missionMap.runJavaScript(script)
    }
    function centerMap(lat, lon, zoom) {
        runMap("window.centerMissionMap(" + Number(lat) + "," + Number(lon) + "," + Number(zoom) + ")")
    }
    function applyMapStyle() {
        runMap("window.setMissionStyle(" + JSON.stringify(mapStyle) + ")")
    }
    function fitMission() {
        runMap("window.fitMissionMap(" + JSON.stringify({
            home: plan.home || null,
            end: plan.end || plan.home || null,
            boundary: plan.boundary || [],
            waypoints: plan.waypoints || []
        }) + "," + startLatitude + "," + startLongitude + "," + startZoom + ")")
    }
    function syncNameFromPlan() {
        nameField.text = String((backendState.missionDraft || ({})).name || "")
    }
    function syncMap() {
        if (!webMapReady) return
        const currentPlan = backendState.missionDraft || ({})
        runMap("window.setOfflineTileTemplate(" + JSON.stringify(String(currentPlan.offline_tile_template || "")) + ")")
        runMap("window.setMissionData(" + JSON.stringify({
            home: currentPlan.home || null,
            end: currentPlan.end || currentPlan.home || null,
            end_is_home: !currentPlan.end,
            boundary: currentPlan.boundary || [],
            waypoints: currentPlan.waypoints || [],
            planned_route: currentPlan.planned_route || [],
            return_route: currentPlan.return_route || [],
            movement_mode: currentPlan.movement_mode || "manual",
            edit_mode: root.editMode
        }) + ")")
        applyMapStyle()
        const planId = String(currentPlan.id || "")
        if (planId !== lastFittedPlanId) {
            lastFittedPlanId = planId
            fitMission()
        }
    }
    function handleMapClick(lat, lon) {
        if (tool === "select") return
        if (editMode === "boundary" && tool !== "boundary" && tool !== "move") return
        if (editMode === "points" && tool === "boundary") return
        if (tool === "move") {
            if (editMode === "boundary" && selectedKind !== "boundary") return
            if (editMode === "points" && selectedKind === "boundary") return
            if (selectedKind) backendObject.updateMissionPoint(selectedKind, selectedIndex, lat, lon)
        } else backendObject.addMissionPoint(tool, lat, lon)
    }
    function selectedPoint() {
        if (selectedKind === "home") return plan.home
        if (selectedKind === "end") return plan.end || plan.home
        let values = plan[selectedKind] || []
        return selectedIndex >= 0 && selectedIndex < values.length ? values[selectedIndex] : null
    }
    function positionFieldsValid() {
        if (!latField.text.trim() || !lonField.text.trim()) return false
        const lat = Number(latField.text), lon = Number(lonField.text)
        return isFinite(lat) && isFinite(lon) && lat >= -90 && lat <= 90 && lon >= -180 && lon <= 180
    }
    function selectPoint(kind, index) {
        selectedKind = kind
        selectedIndex = index
        let point = selectedPoint()
        if (point) {
            latField.text = Number(point.lat).toFixed(7)
            lonField.text = Number(point.lon).toFixed(7)
            speedField.text = String(point.speed_percent || 50)
            holdField.text = String(point.hold_seconds || 0)
        } else {
            latField.text = ""
            lonField.text = ""
        }
    }
    function centerOnBuoy() {
        let pieces = String(backendState.currentLocation || "").split(",")
        if (pieces.length !== 2) return
        let lat = Number(pieces[0]), lon = Number(pieces[1])
        if (isFinite(lat) && isFinite(lon) && Math.abs(lat) <= 90 && Math.abs(lon) <= 180) {
            centerMap(lat, lon, 15)
        }
    }
    onMissionRevisionChanged: {
        if (!missionReady) return
        const currentPlan = backendState.missionDraft || ({})
        syncMap()
        if (currentPlan.movement_mode === "automatic") {
            if (toolPicker && toolPicker.currentIndex > 3) toolPicker.currentIndex = 0
            if (selectedKind === "waypoints") selectedKind = ""
        }
    }
    Component.onCompleted: { missionReady = true; syncNameFromPlan(); syncMap() }

    QtObject {
        id: missionBridge
        WebChannel.id: "missionBridge"
        function mapReady() { root.webMapReady = true; root.syncMap() }
        function mapClicked(lat, lon) { root.handleMapClick(lat, lon) }
        function pointSelected(kind, index) { root.selectPoint(kind, index) }
        function zoomChanged(zoom) {}
    }
    WebChannel { id: missionChannel; registeredObjects: [missionBridge] }

    Rectangle { anchors.fill: parent; radius: 12; color: "#0b1119"; border.color: "#395166" }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 14
        spacing: 9

        RowLayout {
            Layout.fillWidth: true
            Text { text: "Create Mission"; color: root.ink; font.pixelSize: 23; font.bold: true }
            Item { Layout.fillWidth: true }
            MissionButton { text: "New"; onClicked: { backendObject.newMission(); root.selectedKind = ""; Qt.callLater(root.syncNameFromPlan) } }
            ComboBox {
                id: missionPicker
                Layout.preferredWidth: 180
                model: backendState.missionList || []
                textRole: "name"
                valueRole: "id"
                displayText: currentIndex < 0 ? "Saved missions" : currentText
            }
            MissionButton { text: "Open"; enabled: missionPicker.currentIndex >= 0; onClicked: { backendObject.loadMission(missionPicker.currentValue); root.selectedKind = ""; Qt.callLater(root.syncNameFromPlan) } }
            MissionButton { text: "Save"; onClicked: { backendObject.setMissionName(nameField.text); backendObject.saveMission(); Qt.callLater(root.syncNameFromPlan) } }
            MissionButton { text: "Delete"; enabled: (backendState.missionList || []).some(p => p.id === plan.id); onClicked: deleteDialog.open() }
        }
        Text {
            Layout.fillWidth: true
            text: "Plans are edited and saved locally. Upload saved plans from Mission Control; TX12 mission start is reserved on SB/CH7, but waypoint steering is not implemented yet."
            color: "#91b9d2"; font.pixelSize: 12; wrapMode: Text.WordWrap
        }

        RowLayout {
            Layout.fillWidth: true; Layout.fillHeight: true; spacing: 10
            ColumnLayout {
                Layout.fillWidth: true; Layout.fillHeight: true; spacing: 6
                RowLayout {
                    Layout.fillWidth: true
                    ButtonGroup { id: mapStyleButtons; exclusive: true }
                    MissionButton { text: "Edit boundary"; checkable: true; checked: root.editMode === "boundary"; onClicked: { root.editMode = "boundary"; toolPicker.currentIndex = 0; root.tool = "boundary"; root.syncMap() } }
                    MissionButton { text: "Mission points"; checkable: true; checked: root.editMode === "points"; onClicked: { root.editMode = "points"; toolPicker.currentIndex = 0; root.tool = "select"; root.syncMap() } }
                    ComboBox {
                        id: toolPicker
                        Layout.preferredWidth: 155
                        model: root.editMode === "boundary" ? ["Add boundary", "Move selected"] :
                               plan.movement_mode === "automatic" ? ["Select / pan", "Set home", "Set end"] :
                                   ["Select / pan", "Set home", "Set end", "Add movement", "Move selected"]
                        onCurrentIndexChanged: root.tool = root.editMode === "boundary" ? (currentIndex === 0 ? "boundary" : "move") :
                            (plan.movement_mode === "automatic" ? ["select", "home", "end"][currentIndex] : ["select", "home", "end", "waypoints", "move"][currentIndex])
                    }
                    MissionButton { text: "Buoy"; onClicked: root.centerOnBuoy() }
                    MissionButton {
                        text: "Home"; enabled: !!plan.home
                        onClicked: root.centerMap(plan.home.lat, plan.home.lon, 16)
                    }
                    MissionButton { text: "Fit mission"; onClicked: root.fitMission() }
                    Item { Layout.fillWidth: true }
                    MissionButton { text: "Plan settings"; onClicked: root.planPanelOpen = true }
                    MissionButton {
                        text: "Satellite"
                        ButtonGroup.group: mapStyleButtons
                        checkable: true
                        checked: root.mapStyle === "satellite"
                        onClicked: { root.mapStyle = "satellite"; root.applyMapStyle() }
                    }
                    MissionButton {
                        text: "Terrain"
                        ButtonGroup.group: mapStyleButtons
                        checkable: true
                        checked: root.mapStyle === "terrain"
                        onClicked: { root.mapStyle = "terrain"; root.applyMapStyle() }
                    }
                }
                RowLayout {
                    Layout.fillWidth: true
                    Text { text: "Click to place · drag to pan · scroll or pinch to zoom"; color: root.muted; font.pixelSize: 10 }
                    Item { Layout.fillWidth: true }
                    MissionButton { text: "−"; onClicked: root.runMap("window.zoomMissionMap(-1)") }
                    MissionButton { text: "+"; onClicked: root.runMap("window.zoomMissionMap(1)") }
                }
                WebEngineView {
                    id: missionMap
                    objectName: "missionMap"
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    url: Qt.resolvedUrl("mission_map.html")
                    webChannel: missionChannel
                    settings.localContentCanAccessRemoteUrls: true
                }
                Text {
                    Layout.fillWidth: true
                    text: plan.offline_tiles_available
                        ? "Offline USGS map ready for this mission (zoom 10–16). " + root.mapHint
                        : root.mapHint + ". Saving a mission prepares compact USGS tiles for its boundary and route; internet is needed to download them."
                    color: root.muted; font.pixelSize: 10; wrapMode: Text.WordWrap
                }
                Text { Layout.fillWidth: true; text: "Boundary edges uncross as you edit. Points within 2 m of another boundary point are ignored; review the shaded area before saving."; color: root.muted; font.pixelSize: 10; wrapMode: Text.WordWrap }
                Text { Layout.fillWidth: true; text: "Home and End may be outside the survey boundary. If End is unset, the mission ends at Home. Dashed final travel is a preview; routing outside the survey is not planned yet."; color: root.muted; font.pixelSize: 10; wrapMode: Text.WordWrap }
            }
        }
    }

    Rectangle {
        id: planDrawer
        visible: root.planPanelOpen
        z: 5
        anchors.top: parent.top
        anchors.bottom: parent.bottom
        anchors.right: parent.right
        width: Math.min(390, root.width * 0.48)
        radius: 10
        color: "#111b26"
        border.color: "#557088"
        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 10
            spacing: 6
            RowLayout {
                Layout.fillWidth: true
                Text { text: "Plan settings"; color: root.ink; font.pixelSize: 17; font.bold: true }
                Item { Layout.fillWidth: true }
                MissionButton { text: "Close"; onClicked: root.planPanelOpen = false }
            }
            ScrollView {
                Layout.fillWidth: true; Layout.fillHeight: true; clip: true
                ScrollBar.horizontal.policy: ScrollBar.AlwaysOff
                ColumnLayout {
                    width: planDrawer.width - 40; spacing: 8
                    Text { text: "Mission plan"; color: root.ink; font.pixelSize: 17; font.bold: true }
                    TextField { id: nameField; Layout.fillWidth: true; placeholderText: "Mission name"; onEditingFinished: backendObject.setMissionName(text) }
                    GroupBox {
                        title: "Mission estimate"
                        Layout.fillWidth: true
                        ColumnLayout {
                            anchors.fill: parent; spacing: 4
                            Text {
                                Layout.fillWidth: true
                                text: "Survey route: " + Number((backendState.missionEstimate || {}).distance_m || 0).toFixed(0) + " m"
                                color: root.ink; font.pixelSize: 12; font.bold: true
                            }
                            Text {
                                Layout.fillWidth: true
                                text: (backendState.missionEstimate || {}).total_seconds !== null && (backendState.missionEstimate || {}).total_seconds !== undefined
                                      ? "Estimated duration: " + (backendState.missionEstimate.total_seconds >= 3600
                                            ? Math.floor(backendState.missionEstimate.total_seconds / 3600) + " h " + Math.floor((backendState.missionEstimate.total_seconds % 3600) / 60) + " min"
                                            : backendState.missionEstimate.total_seconds >= 60
                                              ? Math.floor(backendState.missionEstimate.total_seconds / 60) + " min"
                                              : Math.ceil(backendState.missionEstimate.total_seconds) + " s") + " (travel + collection stops)"
                                      : "Estimated duration: enter calibrated speed values below"
                                color: root.muted; font.pixelSize: 11; wrapMode: Text.WordWrap
                            }
                            Text {
                                Layout.fillWidth: true
                                text: (backendState.missionEstimate || {}).battery_percent !== null && (backendState.missionEstimate || {}).battery_percent !== undefined
                                      ? "Approx. charge: " + Number(backendState.missionEstimate.battery_ah).toFixed(2) + " Ah (" + Number(backendState.missionEstimate.battery_percent).toFixed(1) + "% of usable capacity)"
                                      : "Approx. charge: enter current and battery calibration below"
                                color: root.muted; font.pixelSize: 11; wrapMode: Text.WordWrap
                            }
                            Text { Layout.fillWidth: true; text: "Route length includes Home to the first point and each planned leg. It excludes the dashed End leg because that return route is only a preview. Estimates are planning aids; water, wind, turns, and load affect actual results."; color: root.muted; font.pixelSize: 10; wrapMode: Text.WordWrap }
                            Text { text: "Calibrated movement speed (m/s)"; color: root.muted; font.pixelSize: 11 }
                            RowLayout {
                                Layout.fillWidth: true; spacing: 3
                                Repeater {
                                    model: [25, 50, 75, 100]
                                    delegate: ColumnLayout {
                                        required property int modelData
                                        Layout.fillWidth: true; spacing: 1
                                        Text { text: modelData + "%"; color: root.muted; font.pixelSize: 9; Layout.alignment: Qt.AlignHCenter }
                                        TextField {
                                            Layout.fillWidth: true; placeholderText: "m/s"; inputMethodHints: Qt.ImhFormattedNumbersOnly
                                            text: plan.calibration && plan.calibration.speed_mps && plan.calibration.speed_mps[String(modelData)] !== null ? String(plan.calibration.speed_mps[String(modelData)]) : ""
                                            onEditingFinished: backendObject.setMissionCalibration("speed_mps", modelData, Number(text))
                                        }
                                    }
                                }
                            }
                            Text { text: "Measured motor current (A) at each speed"; color: root.muted; font.pixelSize: 11 }
                            RowLayout {
                                Layout.fillWidth: true; spacing: 3
                                Repeater {
                                    model: [25, 50, 75, 100]
                                    delegate: ColumnLayout {
                                        required property int modelData
                                        Layout.fillWidth: true; spacing: 1
                                        Text { text: modelData + "%"; color: root.muted; font.pixelSize: 9; Layout.alignment: Qt.AlignHCenter }
                                        TextField {
                                            Layout.fillWidth: true; placeholderText: "A"; inputMethodHints: Qt.ImhFormattedNumbersOnly
                                            text: plan.calibration && plan.calibration.current_a && plan.calibration.current_a[String(modelData)] !== null ? String(plan.calibration.current_a[String(modelData)]) : ""
                                            onEditingFinished: backendObject.setMissionCalibration("current_a", modelData, Number(text))
                                        }
                                    }
                                }
                            }
                            RowLayout {
                                Layout.fillWidth: true
                                Text { text: "Idle current (A)"; color: root.muted; font.pixelSize: 10; Layout.fillWidth: true }
                                TextField { Layout.preferredWidth: 58; placeholderText: "A"; inputMethodHints: Qt.ImhFormattedNumbersOnly; text: plan.calibration && plan.calibration.idle_current_a !== null ? String(plan.calibration.idle_current_a) : ""; onEditingFinished: backendObject.setMissionIdleCurrent(Number(text)) }
                                Text { text: "Usable battery (Ah)"; color: root.muted; font.pixelSize: 10 }
                                TextField { Layout.preferredWidth: 58; placeholderText: "Ah"; inputMethodHints: Qt.ImhFormattedNumbersOnly; text: plan.calibration && plan.calibration.battery_capacity_ah !== null ? String(plan.calibration.battery_capacity_ah) : ""; onEditingFinished: backendObject.setMissionBatteryCapacity(Number(text)) }
                            }
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        Text { text: "Movement"; color: root.muted; font.pixelSize: 12; Layout.fillWidth: true }
                        ComboBox {
                            model: ["Manual", "Automatic"]
                            currentIndex: plan.movement_mode === "automatic" ? 1 : 0
                            onActivated: {
                                toolPicker.currentIndex = 0
                                root.selectedKind = ""
                                backendObject.setMissionMovementMode(currentIndex === 1 ? "automatic" : "manual")
                            }
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        visible: plan.movement_mode === "automatic"
                        Text { text: "Pass spacing (m)"; color: root.muted; font.pixelSize: 11; Layout.fillWidth: true }
                        SpinBox {
                            from: 5; to: 500; stepSize: 5
                            value: plan.auto_spacing_m || 25
                            onValueModified: backendObject.setMissionAutoSpacing(value)
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        visible: plan.movement_mode === "automatic"
                        Text { text: "Boundary clearance (m)"; color: root.muted; font.pixelSize: 11; Layout.fillWidth: true }
                        SpinBox {
                            from: 5; to: 100; stepSize: 1
                            value: plan.auto_clearance_m || 15
                            onValueModified: backendObject.setMissionAutoClearance(value)
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        visible: plan.movement_mode === "automatic"
                        Text { text: "Route speed (%)"; color: root.muted; font.pixelSize: 11; Layout.fillWidth: true }
                        SpinBox {
                            from: 1; to: 100; stepSize: 5
                            value: plan.auto_speed_percent || 50
                            onValueModified: backendObject.setMissionAutoSpeed(value)
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        visible: plan.movement_mode === "automatic"
                        Text { text: "Collection duration (s)"; color: root.muted; font.pixelSize: 11; Layout.fillWidth: true }
                        SpinBox {
                            from: 1; to: 3600; stepSize: 5
                            value: plan.auto_collection_seconds || 30
                            onValueModified: backendObject.setMissionAutoCollectionDuration(value)
                        }
                    }
                    Text {
                        Layout.fillWidth: true
                        text: "Light green circles show the 5 m arrival area for each collection checkpoint. At a checkpoint, the planned sequence is engines off, collect sensors and audio for the set duration, then continue."
                        color: root.muted; font.pixelSize: 11; wrapMode: Text.WordWrap
                    }
                    Text {
                        Layout.fillWidth: true
                        visible: plan.movement_mode === "automatic"
                        text: "Automatic places checkpoints along survey passes and transit points around obstacles. It keeps the route inside the selected boundary clearance and updates after boundary edits. It replaces manual points; switching back to Manual keeps the generated route for editing."
                        color: root.muted; font.pixelSize: 11; wrapMode: Text.WordWrap
                    }
                    Text {
                        Layout.fillWidth: true; text: (backendState.missionDirty ? "Unsaved · " : "") + (backendState.missionMessage || "")
                        color: backendState.missionDirty ? "#ffb454" : "#91b9d2"; wrapMode: Text.WordWrap; font.pixelSize: 11
                    }
                    Text { text: "Points"; color: root.ink; font.pixelSize: 15; font.bold: true }
                    MissionButton {
                        Layout.fillWidth: true
                        text: plan.home ? "Home  " + Number(plan.home.lat).toFixed(5) + ", " + Number(plan.home.lon).toFixed(5) : "Home · place on map"
                        onClicked: root.selectPoint("home", -1)
                    }
                    MissionButton {
                        Layout.fillWidth: true
                        text: plan.end ? "End  " + Number(plan.end.lat).toFixed(5) + ", " + Number(plan.end.lon).toFixed(5) :
                                         (plan.home ? "End · Home (default)" : "End · place Home or set End")
                        onClicked: root.selectPoint("end", -1)
                    }
                    Text { text: "Boundary  (" + (plan.boundary || []).length + ")"; color: root.muted; font.pixelSize: 12 }
                    Repeater {
                        model: plan.boundary || []
                        delegate: MissionButton {
                            required property var modelData
                            required property int index
                            Layout.fillWidth: true
                            text: "B" + (index + 1) + "  " + Number(modelData.lat).toFixed(5) + ", " + Number(modelData.lon).toFixed(5)
                            onClicked: root.selectPoint("boundary", index)
                        }
                    }
                    Text { text: "Measurement and movement targets  (" + (plan.waypoints || []).length + ")"; color: root.muted; font.pixelSize: 12 }
                    Text {
                        Layout.fillWidth: true
                        text: plan.route_generation_error
                            ? String(plan.route_generation_error)
                            : "Safe route: " + (plan.planned_route || []).length + " buoy points · return path " + (plan.return_route || []).length + " points · stays " + (plan.movement_mode === "automatic" ? (plan.auto_clearance_m || 15) : 5) + " m inside the boundary"
                        color: plan.route_generation_error ? "#ff9a9a" : root.muted
                        font.pixelSize: 10; wrapMode: Text.WordWrap
                    }
                    Repeater {
                        model: plan.movement_mode === "automatic" ? (plan.waypoints || []).slice(0, 50) : (plan.waypoints || [])
                        delegate: RowLayout {
                            required property var modelData
                            required property int index
                            Layout.fillWidth: true
                            MissionButton {
                                Layout.fillWidth: true
                                text: (modelData.collect_data === true || (modelData.collect_data === undefined && modelData.hold_seconds > 0) ? "C" : "T") + (index + 1) + "  " + Number(modelData.lat).toFixed(5) + ", " + Number(modelData.lon).toFixed(5)
                                onClicked: root.selectPoint("waypoints", index)
                            }
                            MissionButton { text: "↑"; visible: plan.movement_mode !== "automatic"; enabled: index > 0; onClicked: backendObject.moveMissionWaypoint(index, -1) }
                            MissionButton { text: "↓"; visible: plan.movement_mode !== "automatic"; enabled: index < (plan.waypoints || []).length - 1; onClicked: backendObject.moveMissionWaypoint(index, 1) }
                        }
                    }
                    Text {
                        visible: plan.movement_mode === "automatic" && (plan.waypoints || []).length > 50
                        text: "Showing the first 50 points; the full route is on the map."
                        color: root.muted; font.pixelSize: 11; wrapMode: Text.WordWrap; Layout.fillWidth: true
                    }
                    Text { text: "Selected point"; color: root.ink; font.pixelSize: 15; font.bold: true }
                    Text {
                        text: root.selectedKind ? (root.selectedKind === "home" ? "Home" : root.selectedKind === "end" ? (plan.end ? "End" : "End · using Home") : root.selectedKind === "boundary" ? "Boundary " + (root.selectedIndex + 1) : "Movement " + (root.selectedIndex + 1)) : "Select a point above or on the map"
                        color: root.muted; font.pixelSize: 11
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        TextField { id: latField; Layout.fillWidth: true; placeholderText: "Latitude"; inputMethodHints: Qt.ImhFormattedNumbersOnly }
                        TextField { id: lonField; Layout.fillWidth: true; placeholderText: "Longitude"; inputMethodHints: Qt.ImhFormattedNumbersOnly }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        MissionButton { text: "Update position"; enabled: !!root.selectedKind && root.positionFieldsValid() && (plan.movement_mode !== "automatic" || root.selectedKind !== "waypoints"); onClicked: backendObject.updateMissionPoint(root.selectedKind, root.selectedIndex, Number(latField.text), Number(lonField.text)) }
                        MissionButton { text: "Remove"; enabled: !!root.selectedKind && (root.selectedKind !== "end" || !!plan.end) && (plan.movement_mode !== "automatic" || root.selectedKind !== "waypoints"); onClicked: { backendObject.removeMissionPoint(root.selectedKind, root.selectedIndex); root.selectedKind = "" } }
                    }
                    RowLayout {
                        Layout.fillWidth: true; visible: root.selectedKind === "waypoints" && plan.movement_mode !== "automatic"
                        Text { text: "Speed %"; color: root.muted; font.pixelSize: 11 }
                        TextField { id: speedField; Layout.preferredWidth: 50; validator: IntValidator { bottom: 1; top: 100 } }
                        Text { text: "Collect s"; color: root.muted; font.pixelSize: 11 }
                        TextField { id: holdField; Layout.preferredWidth: 54; validator: IntValidator { bottom: 0; top: 3600 } }
                        MissionButton { text: "Set"; onClicked: backendObject.setMissionWaypointOptions(root.selectedIndex, Number(speedField.text), Number(holdField.text)) }
                    }
                    ColumnLayout {
                        visible: root.selectedKind === "waypoints"
                        Layout.fillWidth: true
                        Text { text: "Measurements at this point"; color: root.ink; font.pixelSize: 13; font.bold: true }
                        Repeater {
                            model: [ {key:"water_temperature", label:"Water temperature"}, {key:"depth", label:"Depth"}, {key:"conductivity", label:"Conductivity / TDS"}, {key:"turbidity", label:"Turbidity"}, {key:"air_temperature", label:"Air temperature"} ]
                            delegate: CheckBox {
                                required property var modelData
                                text: modelData.label
                                checked: !root.selectedPoint() || !(root.selectedPoint().measurements instanceof Array) || root.selectedPoint().measurements.indexOf(modelData.key) >= 0
                                onToggled: {
                                    let point = root.selectedPoint()
                                    let selected = point && point.measurements instanceof Array ? point.measurements.slice() : ["water_temperature", "depth", "conductivity", "turbidity", "air_temperature"]
                                    let idx = selected.indexOf(modelData.key)
                                    if (checked && idx < 0) selected.push(modelData.key)
                                    if (!checked && idx >= 0) selected.splice(idx, 1)
                                    backendObject.setMissionWaypointMeasurements(root.selectedIndex, selected)
                                }
                            }
                        }
                        Text { text: "New movement points collect all listed measurements for the default 30 seconds. These choices are saved in the mission plan."; color: root.muted; font.pixelSize: 10; wrapMode: Text.WordWrap }
                    }
                    Text { visible: root.selectedKind === "waypoints" && plan.movement_mode !== "automatic"; text: "Set collection to 0 seconds for a transit point."; color: root.muted; font.pixelSize: 11; wrapMode: Text.WordWrap; Layout.fillWidth: true }
                    Text { text: "Planned failsafes"; color: root.ink; font.pixelSize: 15; font.bold: true }
                    Text { text: "Saved in the plan; these do not affect the buoy yet."; color: root.muted; font.pixelSize: 11; wrapMode: Text.WordWrap; Layout.fillWidth: true }
                    Repeater {
                        model: [ { label: "ELRS link loss", key: "link_loss" }, { label: "Boundary breach", key: "boundary_breach" }, { label: "Low battery", key: "low_battery" } ]
                        delegate: RowLayout {
                            required property var modelData
                            Layout.fillWidth: true
                            Text { text: modelData.label; color: root.muted; font.pixelSize: 11; Layout.fillWidth: true }
                            ComboBox {
                                model: ["Return home", "Stop"]
                                currentIndex: (plan.failsafes || {})[modelData.key] === "stop" ? 1 : 0
                                onActivated: backendObject.setMissionFailsafe(modelData.key, currentIndex === 1 ? "stop" : "return_home")
                            }
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        Text { text: "Low battery at %"; color: root.muted; font.pixelSize: 11; Layout.fillWidth: true }
                        SpinBox { from: 5; to: 80; value: (plan.failsafes || {}).battery_threshold_percent || 20; onValueModified: backendObject.setMissionBatteryThreshold(value) }
                    }
                    Text { text: "Plan checks"; color: root.ink; font.pixelSize: 15; font.bold: true }
                    Text {
                        Layout.fillWidth: true
                        text: (backendState.missionIssues || []).length ? (backendState.missionIssues || []).join("\n") : "Plan geometry and settings are ready for future mission support."
                        color: (backendState.missionIssues || []).length ? "#ffb454" : "#4dca81"; wrapMode: Text.WordWrap; font.pixelSize: 11
                    }
                    Text { text: "Live buoy: " + (backendState.buoyMode || "Unknown") + " · " + (backendState.radioArmStatus || "Unknown") + " · " + (backendState.recoveryStatus || "Unknown"); color: root.muted; font.pixelSize: 11; wrapMode: Text.WordWrap; Layout.fillWidth: true }
                }
            }
        }
    }
    Dialog {
        id: deleteDialog
        title: "Delete local mission?"; modal: true; anchors.centerIn: parent
        standardButtons: Dialog.Yes | Dialog.Cancel
        onAccepted: { backendObject.deleteMission(); root.selectedKind = ""; Qt.callLater(root.syncNameFromPlan) }
        Text { text: "Delete the current saved mission file?"; color: "#222" }
    }
}
