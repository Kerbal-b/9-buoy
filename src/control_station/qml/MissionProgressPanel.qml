import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtWebEngine
import QtWebChannel

Item {
    id: root
    property var backendObject
    property var backendState: ({})
    property var plan: backendState.missionPreviewPlan || ({})
    property var missionRun: backendState.missionProgress || null
    property bool webMapReady: false
    property bool initialBuoyCenterDone: false
    property string observedBuoyMissionId: ""
    property string fittedBuoyMissionId: ""
    property bool initialBuoyScanRequested: false
    property bool uploadWasStarted: false
    property string mapStyle: "terrain"
    property string offlineTileMissionId: ""
    property string currentLocationText: String(backendState.currentLocation || "")
    property string deleteTargetMissionId: ""
    property string deleteTargetMissionName: ""
    readonly property color ink: "#e6ecf2"
    readonly property color muted: "#9eb5c7"

    function currentPosition() {
        const parts = String(backendState.currentLocation || "").split(",")
        if (parts.length !== 2) return null
        const lat = Number(parts[0]), lon = Number(parts[1])
        if (!isFinite(lat) || !isFinite(lon) || Math.abs(lat) > 90 || Math.abs(lon) > 180) return null
        return { lat: lat, lon: lon }
    }
    function buoyMissionTitle(id) {
        const key = String(id || "NONE")
        if (key === "NONE") return "None"
        const item = (backendState.buoyMissionList || []).find(entry => String(entry.id || "").toLowerCase() === key.toLowerCase())
        return item ? String(item.name || item.id) : key
    }
    function currentBuoyMission() {
        const missions = backendState.buoyMissionList || []
        return missions.length ? missions[0] : null
    }
    function currentBuoyRoute() {
        const loadedId = String(backendState.buoyMissionLoadedId || "NONE").toLowerCase()
        const pointsId = String(backendState.buoyMissionPointsId || "NONE").toLowerCase()
        if (loadedId === "none" || loadedId !== pointsId) return []
        return backendState.buoyMissionPoints || []
    }
    function missionStartDistanceMeters() {
        const position = currentPosition()
        const route = currentBuoyRoute()
        if (!position || route.length === 0) return null
        const toRad = Math.PI / 180
        const lat1 = position.lat * toRad
        const lat2 = Number(route[0].lat) * toRad
        const dLat = lat2 - lat1
        const dLon = (Number(route[0].lon) - position.lon) * toRad
        const a = Math.sin(dLat / 2) * Math.sin(dLat / 2)
            + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) * Math.sin(dLon / 2)
        return 6371000 * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a))
    }
    function missionActionSummary() {
        const progress = root.missionRun
        const route = root.currentBuoyRoute()
        if (progress) {
            const index = Number(progress.current_waypoint_index)
            const point = Number.isInteger(index) && index >= 0 ? route[index] : null
            const requested = point && Array.isArray(point.measurements) ? point.measurements : []
            const measurementLabel = requested.length ? requested.join(", ") : "configured measurements"
            const action = String(progress.current_action || progress.action || progress.status || "").toLowerCase()
            let now = String(progress.current_action || progress.action || progress.status || "Mission active")
            let next = String(progress.next_action || "Awaiting the next mission status update")
            if (action.includes("travel") || action.includes("moving") || action.includes("transit")) {
                now = point ? "Traveling to point " + (index + 1) : "Traveling to the next point"
                if (!progress.next_action) next = point && requested.length
                    ? "Take " + measurementLabel + " at point " + (index + 1)
                    : "Begin measurements at the next point"
            } else if (action.includes("measur") || action.includes("collect") || action.includes("sampl")) {
                now = "Taking " + measurementLabel
                if (!progress.next_action) next = "Continue to the next mission point"
            } else if (action.includes("wait") || action.includes("idle")) {
                now = "Waiting"
            }
            return {action: now, next_action: next}
        }
        const missionStatus = String(backendState.radioMissionStatus || "").toLowerCase()
        if (missionStatus.includes("guidance unavailable")) {
            return {action: "Waiting · mission guidance unavailable",
                    next_action: "The current firmware cannot execute mission points"}
        }
        if (String(backendState.scienceExperimentState || "").toUpperCase() === "RECORDING") {
            return {action: "Taking measurements · recording session",
                    next_action: "Waiting for a mission progress update"}
        }
        const loadedId = String(backendState.buoyMissionLoadedId || "NONE")
        if (loadedId.toUpperCase() !== "NONE") {
            return {action: "Waiting · mission loaded",
                    next_action: "Waiting for the buoy to report mission execution"}
        }
        return {action: "Waiting for a mission", next_action: "Upload a mission to the buoy"}
    }
    function syncMap() {
        if (!webMapReady) return
        const loadedId = String(backendState.buoyMissionLoadedId || "NONE")
        const loadedKey = loadedId.toLowerCase()
        if (loadedKey !== observedBuoyMissionId) {
            observedBuoyMissionId = loadedKey
            fittedBuoyMissionId = ""
            initialBuoyCenterDone = false
        }
        if (loadedId !== offlineTileMissionId) {
            offlineTileMissionId = loadedId
            const offlineTemplate = loadedId.toUpperCase() === "NONE" ? "" : backendObject.missionOfflineTileTemplate(loadedId)
            runMap("window.setOfflineTileTemplate(" + JSON.stringify(String(offlineTemplate || "")) + ")")
        }
        const route = currentBuoyRoute()
        const home = backendState.buoyMissionHome || null
        const end = backendState.buoyMissionEnd || null
        const boundary = backendState.buoyMissionBoundary || []
        runMap("window.setMissionData(" + JSON.stringify({
            home: home,
            end: end,
            end_is_home: !end,
            boundary: boundary.map(point => ({lat: Number(point.lat), lon: Number(point.lon)})),
            waypoints: route.map(point => ({
                lat: Number(point.lat), lon: Number(point.lon),
                collect_data: Boolean(point.collect_data),
                measurements: Array.isArray(point.measurements) ? point.measurements : []
            })),
            is_buoy_mission: true,
            movement_mode: "manual",
            current_location: currentPosition(),
            show_boundary_points: false,
            active_index: missionRun ? Number(missionRun.current_waypoint_index) : -1,
            completed_count: missionRun ? Number(missionRun.completed_waypoints) : 0
        }) + ")")
        runMap("window.setBuoyStatus(" + JSON.stringify({
            position: currentPosition(),
            action: missionActionSummary().action,
            next_action: missionActionSummary().next_action
        }) + ")")
        const expectedPoints = Number(backendState.buoyMissionExpectedPointCount || 0)
        if (loadedKey !== "none" && loadedKey && fittedBuoyMissionId !== loadedKey
                && String(backendState.buoyMissionPointsId || "").toLowerCase() === loadedKey
                && expectedPoints > 0 && route.length >= expectedPoints) {
            runMap("window.fitMissionMap(" + JSON.stringify({
                home: home,
                end: end || home,
                boundary: boundary.map(point => ({lat: Number(point.lat), lon: Number(point.lon)})),
                waypoints: route.map(point => ({lat: Number(point.lat), lon: Number(point.lon)}))
            }) + ",37.53188,-122.24802,14)")
            fittedBuoyMissionId = loadedKey
            initialBuoyCenterDone = true
        } else if (loadedKey === "none" && !initialBuoyCenterDone && currentPosition()) {
            initialBuoyCenterDone = true
            centerBuoy()
        }
        runMap("window.setMissionStyle(" + JSON.stringify(mapStyle) + ",\"\")")
    }
    function runMap(script) { if (webMapReady) missionMap.runJavaScript(script) }
    function centerBuoy() {
        const position = currentPosition()
        if (position) runMap("window.centerMissionMap(" + position.lat + "," + position.lon + ",16)")
    }
    function syncMissionPicker() {
        const plans = backendState.missionList || []
        const selectedId = String(backendState.missionPreviewSelectedId || "")
        const index = plans.findIndex(item => String(item.id) === selectedId)
        if (savedMissionPicker.currentIndex !== index) savedMissionPicker.currentIndex = index
    }
    onCurrentLocationTextChanged: syncMap()
    onPlanChanged: syncMap()
    onMissionRunChanged: syncMap()
    onBackendStateChanged: {
        syncMissionPicker()
        syncMap()
        if (uploadMissionDialog.visible && uploadWasStarted && backendState.missionUploadSucceeded) {
            uploadMissionDialog.close()
            uploadWasStarted = false
        }
        if (!initialBuoyScanRequested && backendState.transportMode === "wifi"
                && String(backendState.serialStatus || "").indexOf("Connected") === 0) {
            initialBuoyScanRequested = true
            Qt.callLater(function() { backendObject.refreshBuoyMissionList() })
        }
    }
    Component.onCompleted: syncMap()

    QtObject {
        id: progressBridge
        WebChannel.id: "missionBridge"
        function mapReady() { root.webMapReady = true; root.syncMap() }
        function zoomChanged(zoom) {}
        function mapClicked(lat, lon) {}
        function pointSelected(kind, index) {}
    }
    WebChannel { id: progressChannel; registeredObjects: [progressBridge] }

    Rectangle { anchors.fill: parent; radius: 12; color: "#0b1119"; border.color: "#395166" }
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 6
        spacing: 5
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 104
            radius: 8
            color: "#111b27"
            border.color: "#395166"
            RowLayout {
                anchors.fill: parent
                anchors.margins: 5
                spacing: 6

                Rectangle {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    radius: 8
                    color: "#172535"
                    border.color: "#314b62"
                    RowLayout {
                        anchors.fill: parent
                        anchors.margins: 7
                        spacing: 7
                        ColumnLayout {
                            Layout.fillWidth: true
                            spacing: 1
                            Text { text: "MISSION ON BUOY"; color: "#7f99ad"; font.pixelSize: 9; font.bold: true }
                            Text {
                                Layout.fillWidth: true
                                text: String(backendState.buoyMissionLoadedId || "NONE") === "NONE"
                                    ? "No mission on buoy" : root.buoyMissionTitle(backendState.buoyMissionLoadedId)
                                color: root.ink
                                font.pixelSize: 13
                                font.bold: true
                                elide: Text.ElideRight
                            }
                        }
                        ColumnLayout {
                            Layout.preferredWidth: 122
                            Layout.fillHeight: true
                            spacing: 3
                        MissionButton {
                            text: backendState.missionUploadPending ? "Uploading…" : "Upload mission"
                            enabled: !backendState.missionUploadPending
                                && (backendState.missionList || []).length > 0
                                && backendState.transportMode === "wifi"
                                && String(backendState.serialStatus || "").indexOf("Connected") === 0
                            onClicked: { root.uploadWasStarted = false; uploadMissionDialog.open() }
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            font.pixelSize: 12
                            palette.buttonText: "#e6ecf2"
                            background: Rectangle {
                                radius: 5
                                color: parent.enabled ? (parent.down ? "#244a63" : "#1b3448") : "#17232e"
                                border.color: parent.enabled ? "#55748d" : "#2d3e4b"
                            }
                        }
                        MissionButton {
                            text: "Delete mission"
                            enabled: !backendState.missionUploadPending
                                && String(backendState.buoyMissionLoadedId || "NONE").toUpperCase() !== "NONE"
                                && String(backendState.buoyMissionLoadedId || "").toLowerCase()
                                    !== String(backendState.buoyMissionRunningId || "").toLowerCase()
                                && backendState.transportMode === "wifi"
                                && String(backendState.serialStatus || "").indexOf("Connected") === 0
                            onClicked: {
                                root.deleteTargetMissionId = String(backendState.buoyMissionLoadedId || "")
                                root.deleteTargetMissionName = root.buoyMissionTitle(root.deleteTargetMissionId)
                                deleteBuoyMissionDialog.open()
                            }
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            font.pixelSize: 12
                            palette.buttonText: "#ffffff"
                            background: Rectangle {
                                radius: 5
                                color: parent.enabled ? (parent.down ? "#752e37" : "#552a31") : "#17232e"
                                border.color: parent.enabled ? "#b65962" : "#2d3e4b"
                            }
                        }
                        }
                    }
                }

            }
        }

        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 10
            WebEngineView {
                id: missionMap
                objectName: "missionProgressMap"
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.minimumWidth: 420
                url: Qt.resolvedUrl("mission_map.html")
                webChannel: progressChannel
                settings.localContentCanAccessRemoteUrls: true
            }
        }
    }
    Dialog {
        id: uploadMissionDialog
        title: "Upload mission to buoy"
        modal: true
        dim: true
        anchors.centerIn: parent
        width: Math.min(480, root.width - 32)
        padding: 14
        background: Rectangle { color: "#111b27"; radius: 12; border.color: "#55748d" }
        header: Rectangle {
            implicitHeight: 54
            color: "#172535"
            radius: 12
            Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: 12; color: "#172535" }
            Text {
                anchors.fill: parent
                anchors.leftMargin: 18
                anchors.rightMargin: 18
                text: uploadMissionDialog.title
                color: root.ink
                font.pixelSize: 17
                font.bold: true
                verticalAlignment: Text.AlignVCenter
            }
        }
        contentItem: ColumnLayout {
            spacing: 12
            Text {
                text: "Choose a saved mission to replace the mission stored on the buoy. After the buoy confirms the SD card write, Mission Control will read it back and update the map."
                color: root.ink
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }
            ComboBox {
                id: savedMissionPicker
                Layout.fillWidth: true
                model: backendState.missionList || []
                textRole: "name"
                valueRole: "id"
                displayText: currentIndex < 0 ? "Select a saved mission" : currentText
                onActivated: backendObject.selectMissionPreview(currentValue)
                palette.text: root.ink
                palette.buttonText: root.ink
                palette.window: "#172535"
                palette.base: "#172535"
                palette.highlight: "#28516b"
                background: Rectangle { radius: 6; color: "#172535"; border.color: "#55748d" }
                indicator: Text {
                    text: "v"
                    color: root.ink
                    font.pixelSize: 15
                    anchors.right: parent.right
                    anchors.rightMargin: 10
                    anchors.verticalCenter: parent.verticalCenter
                }
                contentItem: Text {
                    leftPadding: 10
                    rightPadding: 32
                    text: savedMissionPicker.displayText
                    color: root.ink
                    font.pixelSize: 12
                    verticalAlignment: Text.AlignVCenter
                    elide: Text.ElideRight
                }
                delegate: ItemDelegate {
                    width: savedMissionPicker.width
                    contentItem: Text {
                        text: modelData.name || modelData.id
                        color: root.ink
                        font.pixelSize: 12
                        verticalAlignment: Text.AlignVCenter
                        elide: Text.ElideRight
                    }
                    highlighted: savedMissionPicker.highlightedIndex === index
                    background: Rectangle {
                        color: parent.highlighted ? "#28516b" : "#172535"
                    }
                }
                popup: Popup {
                    y: savedMissionPicker.height - 1
                    width: savedMissionPicker.width
                    padding: 1
                    implicitHeight: Math.min(contentItem.implicitHeight, 240) + padding * 2
                    contentItem: ListView {
                        clip: true
                        implicitHeight: contentHeight
                        model: savedMissionPicker.popup.visible ? savedMissionPicker.delegateModel : null
                        currentIndex: savedMissionPicker.highlightedIndex
                    }
                    background: Rectangle { color: "#172535"; radius: 6; border.color: "#55748d" }
                }
            }
            Text {
                text: backendState.missionPreviewIsDraft === false
                    ? String((backendState.missionPreviewPlan || {}).name || "") + " · "
                        + ((backendState.missionPreviewPlan || {}).planned_route || []).length + " planned route points"
                    : "Select one of your saved missions to continue."
                color: root.muted
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }
            Text {
                visible: String(backendState.missionMessage || "").length > 0
                text: String(backendState.missionMessage || "")
                color: root.muted
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }
        }
        footer: RowLayout {
            spacing: 8
            Item { Layout.fillWidth: true }
            MissionButton {
                text: "Cancel"
                palette.buttonText: root.ink
                background: Rectangle { radius: 6; color: parent.down ? "#202f3d" : "#17232e"; border.color: "#43596b" }
                onClicked: uploadMissionDialog.close()
            }
            MissionButton {
                text: backendState.missionUploadPending ? "Uploading" : "Upload mission"
                enabled: !backendState.missionUploadPending
                    && backendState.missionPreviewIsDraft === false
                    && backendState.transportMode === "wifi"
                    && String(backendState.sdCardStatus || "").indexOf("ready") >= 0
                    && String(backendState.missionPreviewSelectedId || "") !== ""
                    && ((backendState.missionPreviewPlan || {}).planned_route || []).length > 0
                    && ((backendState.missionPreviewPlan || {}).planned_route || []).length <= 64
                    && String(backendState.serialStatus || "").indexOf("Connected") === 0
                palette.buttonText: "#ffffff"
                background: Rectangle { radius: 6; color: parent.enabled ? (parent.down ? "#23604d" : "#174536") : "#17232e"; border.color: parent.enabled ? "#3e9473" : "#2d3e4b" }
                onClicked: {
                    root.uploadWasStarted = true
                    backendObject.uploadMission()
                    // Keep the dialog open until the buoy mission has been read back.
                }
            }
        }
    }
    Dialog {
        id: deleteBuoyMissionDialog
        title: "Delete mission from SD card?"
        modal: true
        dim: true
        anchors.centerIn: parent
        width: Math.min(440, root.width - 32)
        padding: 14
        background: Rectangle { color: "#111b27"; radius: 12; border.color: "#55748d" }
        header: Rectangle {
            implicitHeight: 54
            color: "#172535"
            radius: 12
            Rectangle { anchors.bottom: parent.bottom; width: parent.width; height: 12; color: "#172535" }
            Text {
                anchors.fill: parent
                anchors.leftMargin: 18
                anchors.rightMargin: 18
                text: deleteBuoyMissionDialog.title
                color: root.ink
                font.pixelSize: 17
                font.bold: true
                verticalAlignment: Text.AlignVCenter
            }
        }
        contentItem: Text {
            text: "Delete \"" + root.deleteTargetMissionName + "\" from the buoy SD card? This cannot be undone."
            color: root.ink
            wrapMode: Text.WordWrap
            font.pixelSize: 13
        }
        footer: RowLayout {
            spacing: 8
            Item { Layout.fillWidth: true }
            MissionButton {
                text: "Cancel"
                palette.buttonText: root.ink
                background: Rectangle { radius: 6; color: parent.down ? "#202f3d" : "#17232e"; border.color: "#43596b" }
                onClicked: deleteBuoyMissionDialog.close()
            }
            MissionButton {
                text: "Delete mission"
                palette.buttonText: "#ffffff"
                background: Rectangle { radius: 6; color: parent.down ? "#752e37" : "#9d3845"; border.color: "#e47b84" }
                onClicked: {
                    backendObject.deleteBuoyMission(root.deleteTargetMissionId)
                    deleteBuoyMissionDialog.close()
                }
            }
        }
    }
}
