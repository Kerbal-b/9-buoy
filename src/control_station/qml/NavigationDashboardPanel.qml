import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Item {
    id: root
    property var backendObject
    property var backendState: ({})

    readonly property color ink: "#eef5f9"
    readonly property color muted: "#91a9b9"
    readonly property color edge: "#2a4051"
    readonly property color card: "#111e2a"
    readonly property color cyan: "#65d4da"
    readonly property color blue: "#75baff"
    readonly property color amber: "#f2bb74"
    readonly property color green: "#78ddb2"
    readonly property bool gpsFix: Boolean(backendState.gpsFixValid)
    readonly property bool gpsQualityKnown: backendState.gpsSatelliteCount !== undefined && backendState.gpsSatelliteCount !== null
    readonly property bool positionKnown: String(backendState.currentLocation || "").indexOf("Unknown") !== 0
        && String(backendState.currentLocation || "").split(",").length >= 2
    readonly property bool imuAvailable: backendState.imuAccel !== undefined && backendState.imuAccel !== "N/A"
    readonly property bool tiltHeadingAvailable: backendState.magnetometerTiltHeadingDeg !== undefined && backendState.magnetometerTiltHeadingDeg !== null
    readonly property bool rawHeadingAvailable: backendState.magnetometerHeadingDeg !== undefined && backendState.magnetometerHeadingDeg !== null
    readonly property bool headingAvailable: tiltHeadingAvailable || rawHeadingAvailable
    readonly property real heading: tiltHeadingAvailable ? Number(backendState.magnetometerTiltHeadingDeg) : Number(backendState.magnetometerHeadingDeg || 0)
    readonly property bool gpsQualityGood: gpsFix && backendState.gpsSatelliteCount !== null && backendState.gpsSatelliteCount !== undefined
        && Number(backendState.gpsSatelliteCount) >= Number(backendState.gpsMinimumSatellites || 6)
        && (backendState.gpsHdop === null || backendState.gpsHdop === undefined || Number(backendState.gpsHdop) <= 2.5)
    readonly property string compassCanvasKey: [heading, headingAvailable, backendState.navigationGpsCourseAvailable, backendState.navigationGpsCourseDeg].join("|")
    readonly property string horizonCanvasKey: [backendState.navigationRollDeg, backendState.navigationPitchDeg, imuAvailable].join("|")

    function connected() { return String(backendState.serialStatus || "").indexOf("Connected") === 0 }
    function number(value, digits) { return value === undefined || value === null ? "—" : Number(value).toFixed(digits) }
    function coordinate(index) {
        const parts = String(backendState.currentLocation || "").split(",")
        if (!positionKnown || parts.length < 2) return "—"
        const value = Number(parts[index].trim())
        return Number.isFinite(value) ? value.toFixed(6) + "°" : "—"
    }
    function headingName() {
        if (!headingAvailable) return "WAITING FOR COMPASS"
        return tiltHeadingAvailable ? "TILT CORRECTED HEADING" : "FLAT MAGNETIC HEADING"
    }
    function compassStatus() {
        if (!headingAvailable) return "No magnetometer reading"
        if (!tiltHeadingAvailable) return "Set hull level in the attitude instrument for tilt correction"
        return backendState.magnetometerTiltStatus || "Tilt correction active"
    }
    function headingDifference() {
        if (!backendState.navigationGpsCourseAvailable || !headingAvailable) return "—"
        return number(backendState.navigationHeadingCourseOffsetDeg, 1) + "°"
    }

    onCompassCanvasKeyChanged: if (compassCanvas) compassCanvas.requestPaint()
    onHorizonCanvasKeyChanged: if (horizonCanvas) horizonCanvas.requestPaint()

    component ActionButton: Button {
        id: actionButton
        property bool primary: false
        property bool destructive: false
        implicitHeight: 34
        implicitWidth: Math.max(34, actionButtonText.implicitWidth + 22)
        background: Rectangle {
            radius: 8
            color: !actionButton.enabled ? "#172431" : actionButton.destructive ? "#392630" : actionButton.primary ? "#175662" : actionButton.hovered ? "#263c4d" : "#1c2d3b"
            border.color: !actionButton.enabled ? "#263643" : actionButton.destructive ? "#85505c" : actionButton.primary ? "#3c9ca7" : "#385064"
        }
        contentItem: Text {
            id: actionButtonText
            text: actionButton.text
            color: !actionButton.enabled ? "#657b8b" : actionButton.destructive ? "#ffc5c9" : actionButton.primary ? "#e8ffff" : "#dbe8ef"
            font.pixelSize: 11
            font.bold: true
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
        }
    }

    ScrollView {
        id: scroll
        anchors.fill: parent
        clip: true
        ScrollBar.horizontal.policy: ScrollBar.AlwaysOff

        Column {
            x: 18
            y: 16
            width: Math.max(280, scroll.availableWidth - 36)
            spacing: 12

            RowLayout {
                width: parent.width
                spacing: 12
                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 2
                    Text { text: "Navigation"; color: root.ink; font.pixelSize: 24; font.bold: true }
                    Text { text: "Position · heading · hull attitude"; color: root.muted; font.pixelSize: 12 }
                }
                ActionButton {
                    text: backendState.imuStreamEnabled ? "Stop IMU stream" : "Start IMU stream"
                    enabled: root.connected()
                    onClicked: if (backendObject) backendObject.setImuStreamEnabled(!backendState.imuStreamEnabled)
                }
            }

            RowLayout {
                width: parent.width
                spacing: 10
                Text {
                    Layout.fillWidth: true
                    text: backendState.testSessionActive
                        ? "TEST LOG  ·  " + Number(backendState.testSessionSampleCount || 0) + " samples  ·  "
                            + Number(backendState.testSessionImuRateHz || 0).toFixed(1) + " Hz  ·  "
                            + Number(backendState.testSessionImuLossPercent || 0).toFixed(1) + "% loss"
                        : (backendState.testSessionStatus || "Test log ready")
                    color: backendState.testSessionActive ? root.green : root.muted
                    font.pixelSize: 10
                    elide: Text.ElideRight
                }
                ActionButton {
                    text: backendState.testSessionActive ? "Stop test log" : "Record test log"
                    primary: !backendState.testSessionActive
                    enabled: root.connected() && (backendState.testSessionActive || backendState.imuStreamEnabled)
                    onClicked: {
                        if (!backendObject) return
                        if (backendState.testSessionActive) backendObject.stopTestSession()
                        else backendObject.startTestSession()
                    }
                }
            }

            GridLayout {
                width: parent.width
                columns: width >= 900 ? 3 : width >= 700 ? 2 : 1
                columnSpacing: 12
                rowSpacing: 12

                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 362
                    radius: 16
                    color: root.card
                    border.color: root.edge
                    Column {
                        anchors.fill: parent
                        anchors.margins: 18
                        spacing: 10
                        RowLayout {
                            width: parent.width
                            Text { text: "GPS POSITION"; color: root.blue; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1.2; Layout.fillWidth: true }
                            Rectangle {
                                width: 68; height: 23; radius: 11
                                color: root.gpsFix ? "#173b34" : "#3a3030"
                                border.color: root.gpsFix ? "#346e58" : "#68504a"
                                Text { anchors.centerIn: parent; text: root.gpsFix ? "FIX" : root.gpsQualityKnown ? "NO FIX" : "UNKNOWN"; color: root.gpsFix ? root.green : root.amber; font.pixelSize: 10; font.bold: true }
                            }
                        }
                        Text { text: root.coordinate(0); color: root.ink; font.pixelSize: 22; font.bold: true; font.family: "Consolas"; elide: Text.ElideRight; width: parent.width }
                        Text { text: root.coordinate(1); color: root.ink; font.pixelSize: 22; font.bold: true; font.family: "Consolas"; elide: Text.ElideRight; width: parent.width }
                        Text { text: root.positionKnown ? (backendState.telemetrySource || "GPS receiver") : "Waiting for a valid position"; color: root.muted; font.pixelSize: 11 }
                        Rectangle { width: parent.width; height: 1; color: root.edge }
                        RowLayout {
                            width: parent.width
                            spacing: 10
                            Canvas {
                                width: 28; height: 28
                                onPaint: {
                                    const c = getContext("2d"); c.clearRect(0, 0, width, height)
                                    c.strokeStyle = root.blue; c.fillStyle = root.blue; c.lineWidth = 2
                                    c.save(); c.translate(14, 14); c.rotate(-0.45)
                                    c.strokeRect(-5, -5, 10, 10)
                                    c.beginPath(); c.moveTo(-12, -8); c.lineTo(-7, -8); c.lineTo(-7, 8); c.lineTo(-12, 8)
                                    c.moveTo(12, -8); c.lineTo(7, -8); c.lineTo(7, 8); c.lineTo(12, 8)
                                    c.moveTo(0, 5); c.lineTo(0, 11); c.stroke(); c.restore()
                                }
                            }
                            ColumnLayout {
                                spacing: 0; Layout.fillWidth: true
                                Text { text: backendState.gpsSatelliteCount === undefined || backendState.gpsSatelliteCount === null ? "—" : String(backendState.gpsSatelliteCount); color: root.ink; font.pixelSize: 23; font.bold: true }
                                Text { text: "SATELLITES"; color: root.muted; font.pixelSize: 9; font.bold: true; font.letterSpacing: 1 }
                            }
                            ColumnLayout {
                                spacing: 0
                                Text { text: root.number(backendState.gpsHdop, 1); color: root.ink; font.pixelSize: 21; font.bold: true }
                                Text { text: "HDOP"; color: root.muted; font.pixelSize: 9; font.bold: true; font.letterSpacing: 1 }
                            }
                        }
                        Text {
                            width: parent.width; height: 28
                            text: backendState.gpsQualityStatus || "Waiting for GPS data"
                            color: root.gpsQualityGood ? root.green : root.amber
                            font.pixelSize: 11; elide: Text.ElideRight; verticalAlignment: Text.AlignVCenter
                        }
                        RowLayout {
                            width: parent.width; spacing: 12
                            ColumnLayout {
                                spacing: 1; Layout.fillWidth: true
                                Text { text: "SPEED OVER GROUND"; color: root.muted; font.pixelSize: 9; font.bold: true }
                                Text { text: backendState.navigationGpsSpeedAvailable ? root.number(backendState.navigationGpsSpeedMps, 2) + " m/s" : "—"; color: root.ink; font.pixelSize: 17; font.bold: true }
                            }
                            ColumnLayout {
                                spacing: 1
                                Text { text: "COURSE OVER GROUND"; color: root.muted; font.pixelSize: 9; font.bold: true }
                                Text { text: backendState.navigationGpsCourseAvailable ? root.number(backendState.navigationGpsCourseDeg, 0) + "°" : "—"; color: root.blue; font.pixelSize: 17; font.bold: true }
                            }
                        }
                        RowLayout {
                            width: parent.width; spacing: 6
                            Text { text: "Quality target"; color: root.muted; font.pixelSize: 10; Layout.fillWidth: true }
                            ActionButton { text: "−"; enabled: Number(backendState.gpsMinimumSatellites || 6) > 4; onClicked: if (backendObject) backendObject.setGpsMinimumSatellites(Number(backendState.gpsMinimumSatellites || 6) - 1) }
                            Text { text: String(backendState.gpsMinimumSatellites || 6) + " sats"; color: root.ink; font.pixelSize: 11; font.bold: true; horizontalAlignment: Text.AlignHCenter; Layout.preferredWidth: 48 }
                            ActionButton { text: "+"; enabled: Number(backendState.gpsMinimumSatellites || 6) < 12; onClicked: if (backendObject) backendObject.setGpsMinimumSatellites(Number(backendState.gpsMinimumSatellites || 6) + 1) }
                        }
                    }
                }

                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 362
                    radius: 16
                    color: root.card
                    border.color: root.edge
                    Column {
                        anchors.fill: parent
                        anchors.margins: 18
                        spacing: 4
                        RowLayout {
                            width: parent.width
                            Text { text: "MAGNETIC HEADING"; color: root.cyan; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1.2; Layout.fillWidth: true }
                            Text { text: root.tiltHeadingAvailable ? "TILT CORRECTED" : "FLAT"; color: root.tiltHeadingAvailable ? root.green : root.amber; font.pixelSize: 9; font.bold: true }
                        }
                        Item {
                            width: parent.width; height: 244
                            Canvas {
                                id: compassCanvas
                                width: Math.min(parent.width, 232); height: width
                                anchors.centerIn: parent
                                onWidthChanged: requestPaint()
                                onPaint: {
                                    const c = getContext("2d"); c.clearRect(0, 0, width, height)
                                    const cx = width / 2, cy = height / 2, radius = width * 0.43
                                    c.fillStyle = "#0b1722"; c.beginPath(); c.arc(cx, cy, radius, 0, Math.PI * 2); c.fill()
                                    c.strokeStyle = "#284252"; c.lineWidth = 2; c.beginPath(); c.arc(cx, cy, radius, 0, Math.PI * 2); c.stroke()
                                    c.save(); c.translate(cx, cy); c.rotate(-(root.headingAvailable ? root.heading : 0) * Math.PI / 180)
                                    c.textAlign = "center"; c.textBaseline = "middle"
                                    for (let degree = 0; degree < 360; degree += 10) {
                                        const angle = (degree - 90) * Math.PI / 180
                                        const cardinal = degree % 90 === 0, major = degree % 30 === 0
                                        c.strokeStyle = cardinal ? "#b5d9e5" : major ? "#7296a7" : "#476374"
                                        c.lineWidth = cardinal ? 2.5 : 1
                                        c.beginPath(); c.moveTo(Math.cos(angle) * (radius - (cardinal ? 16 : major ? 12 : 7)), Math.sin(angle) * (radius - (cardinal ? 16 : major ? 12 : 7)))
                                        c.lineTo(Math.cos(angle) * (radius - 3), Math.sin(angle) * (radius - 3)); c.stroke()
                                        if (major) {
                                            c.fillStyle = degree === 0 ? "#78dadd" : "#c8d9e2"
                                            c.font = cardinal ? "bold 14px sans-serif" : "10px sans-serif"
                                            const label = degree === 0 ? "N" : degree === 90 ? "E" : degree === 180 ? "S" : degree === 270 ? "W" : String(degree)
                                            c.save()
                                            c.translate(Math.cos(angle) * (radius - 32), Math.sin(angle) * (radius - 32))
                                            c.rotate((root.headingAvailable ? root.heading : 0) * Math.PI / 180)
                                            c.fillText(label, 0, 0)
                                            c.restore()
                                        }
                                    }
                                    c.restore()
                                    c.fillStyle = "#6ce0de"; c.beginPath(); c.moveTo(cx, cy - radius - 5); c.lineTo(cx - 7, cy - radius + 7); c.lineTo(cx + 7, cy - radius + 7); c.closePath(); c.fill()
                                    if (root.headingAvailable && backendState.navigationGpsCourseAvailable) {
                                        const relative = (Number(backendState.navigationGpsCourseDeg) - root.heading - 90) * Math.PI / 180
                                        const x = cx + Math.cos(relative) * (radius + 1), y = cy + Math.sin(relative) * (radius + 1)
                                        c.fillStyle = "#75baff"; c.beginPath(); c.arc(x, y, 5, 0, Math.PI * 2); c.fill()
                                    }
                                }
                            }
                            Column {
                                anchors.centerIn: parent; spacing: 0
                                Text { text: root.headingAvailable ? root.number(root.heading, 0) + "°" : "—"; color: root.ink; font.pixelSize: 34; font.bold: true; anchors.horizontalCenter: parent.horizontalCenter }
                                Text { text: root.headingAvailable ? "MAGNETIC" : "NO DATA"; color: root.muted; font.pixelSize: 9; font.bold: true; font.letterSpacing: 1; anchors.horizontalCenter: parent.horizontalCenter }
                            }
                        }
                        Text { text: root.compassStatus(); color: root.tiltHeadingAvailable ? root.green : root.amber; font.pixelSize: 10; elide: Text.ElideRight; width: parent.width }
                        Text { text: backendState.magnetometerRaw || "X —  ·  Y —  ·  Z —"; color: root.muted; font.pixelSize: 10; elide: Text.ElideRight; width: parent.width }
                    }
                }

                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 390
                    radius: 16
                    color: root.card
                    border.color: root.edge
                    Column {
                        anchors.fill: parent
                        anchors.margins: 18
                        spacing: 5
                        RowLayout {
                            width: parent.width
                            Text { text: "HULL ATTITUDE"; color: root.amber; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1.2; Layout.fillWidth: true }
                            Text { text: root.imuAvailable ? "IMU LIVE" : "NO IMU"; color: root.imuAvailable ? root.green : root.amber; font.pixelSize: 9; font.bold: true }
                        }
                        Item {
                            width: parent.width; height: 213
                            Canvas {
                                id: horizonCanvas
                                width: Math.min(parent.width, 204); height: width
                                anchors.centerIn: parent
                                onWidthChanged: requestPaint()
                                onPaint: {
                                    const c = getContext("2d"); c.clearRect(0, 0, width, height)
                                    const cx = width / 2, cy = height / 2, r = width * 0.46
                                    const roll = root.imuAvailable ? Math.max(-80, Math.min(80, Number(backendState.navigationRollDeg || 0))) : 0
                                    const pitch = root.imuAvailable ? Math.max(-45, Math.min(45, Number(backendState.navigationPitchDeg || 0))) : 0
                                    c.save(); c.beginPath(); c.arc(cx, cy, r, 0, Math.PI * 2); c.clip()
                                    c.translate(cx, cy); c.rotate(-roll * Math.PI / 180)
                                    const line = -pitch * r / 45
                                    c.fillStyle = "#19394b"; c.fillRect(-r * 2, -r * 2, r * 4, r * 2 + line)
                                    c.fillStyle = "#126376"; c.fillRect(-r * 2, line, r * 4, r * 2)
                                    c.strokeStyle = "#9be8e8"; c.lineWidth = 2; c.beginPath(); c.moveTo(-r * 2, line); c.lineTo(r * 2, line); c.stroke()
                                    c.strokeStyle = "#c2e3e6"; c.lineWidth = 1; c.font = "10px sans-serif"; c.textAlign = "center"
                                    for (let mark = -30; mark <= 30; mark += 10) {
                                        if (mark === 0) continue
                                        const y = line - mark * r / 45, half = mark % 20 === 0 ? 26 : 16
                                        c.beginPath(); c.moveTo(-half, y); c.lineTo(half, y); c.stroke()
                                    }
                                    c.restore()
                                    c.strokeStyle = "#38596b"; c.lineWidth = 3; c.beginPath(); c.arc(cx, cy, r, 0, Math.PI * 2); c.stroke()
                                    c.strokeStyle = "#ffd07c"; c.lineWidth = 3
                                    c.beginPath(); c.moveTo(cx - 55, cy); c.lineTo(cx - 17, cy); c.moveTo(cx + 17, cy); c.lineTo(cx + 55, cy); c.moveTo(cx, cy - 8); c.lineTo(cx, cy + 10); c.stroke()
                                    c.fillStyle = "#ffd07c"; c.beginPath(); c.arc(cx, cy, 3, 0, Math.PI * 2); c.fill()
                                    // Keep live hull tilt visible in the horizon instrument itself.
                                    c.fillStyle = "#e7f4f7"; c.font = "bold 10px sans-serif"; c.textAlign = "left"
                                    c.fillText("R " + (root.imuAvailable ? Number(backendState.navigationRollDeg || 0).toFixed(1) + "°" : "—"), cx - r + 10, cy - r + 17)
                                    c.textAlign = "right"
                                    c.fillText("P " + (root.imuAvailable ? Number(backendState.navigationPitchDeg || 0).toFixed(1) + "°" : "—"), cx + r - 10, cy - r + 17)
                                }
                            }
                        }
                        RowLayout {
                            width: parent.width; spacing: 14
                            ColumnLayout { Layout.fillWidth: true; spacing: 1
                                Text { text: "ROLL"; color: root.muted; font.pixelSize: 9; font.bold: true }
                                Text { text: root.imuAvailable ? root.number(backendState.navigationRollDeg, 1) + "°" : "—"; color: root.ink; font.pixelSize: 19; font.bold: true }
                            }
                            ColumnLayout { Layout.fillWidth: true; spacing: 1
                                Text { text: "PITCH"; color: root.muted; font.pixelSize: 9; font.bold: true }
                                Text { text: root.imuAvailable ? root.number(backendState.navigationPitchDeg, 1) + "°" : "—"; color: root.ink; font.pixelSize: 19; font.bold: true }
                            }
                            ColumnLayout { spacing: 1
                                Text { text: "YAW RATE"; color: root.muted; font.pixelSize: 9; font.bold: true }
                                Text { text: root.imuAvailable ? root.number(backendState.navigationMeasuredYawDps, 1) + "°/s" : "—"; color: root.ink; font.pixelSize: 15; font.bold: true }
                            }
                        }
                        Text { text: root.imuAvailable ? "Linear acceleration   X " + root.number(backendState.navigationMeasuredXG, 3) + " g   ·   Y " + root.number(backendState.navigationMeasuredYG, 3) + " g" : "Waiting for IMU stream"; color: root.muted; font.pixelSize: 10; elide: Text.ElideRight; width: parent.width }
                        RowLayout {
                            width: parent.width; spacing: 8
                            Text {
                                Layout.fillWidth: true
                                text: "Motion · 5 s   " + (backendState.navigationSeaAccelerationRmsG === undefined || backendState.navigationSeaAccelerationRmsG === null ? "—" : Number(backendState.navigationSeaAccelerationRmsG).toFixed(3) + " g") + " accel RMS   ·   " + (backendState.navigationSeaAngularRateRmsDps === undefined || backendState.navigationSeaAngularRateRmsDps === null ? "—" : Number(backendState.navigationSeaAngularRateRmsDps).toFixed(1) + " °/s") + " rotation RMS"
                                color: root.blue; font.pixelSize: 9; elide: Text.ElideRight
                            }
                            ActionButton { text: backendState.navigationLevelCalibrated ? "Set level again" : "Set level"; enabled: root.imuAvailable; onClicked: if (backendObject) backendObject.calibrateNavigationLevel() }
                        }
                    }
                }
            }

            Rectangle {
                width: parent.width; height: 66; radius: 12; color: "#132432"; border.color: root.edge
                RowLayout {
                    anchors.fill: parent; anchors.margins: 14; spacing: 15
                    Rectangle { width: 5; height: 35; radius: 3; color: root.blue }
                    ColumnLayout { Layout.fillWidth: true; spacing: 2
                        Text { text: "COURSE VS HEADING"; color: root.muted; font.pixelSize: 9; font.bold: true; font.letterSpacing: 1 }
                        Text { text: backendState.navigationGpsCourseAvailable && root.headingAvailable ? "Travel direction differs from bow direction by " + root.headingDifference() : "Available after GPS detects movement and the compass reports heading"; color: root.ink; font.pixelSize: 12; elide: Text.ElideRight; Layout.fillWidth: true }
                    }
                    Text { text: root.headingDifference(); color: root.blue; font.pixelSize: 21; font.bold: true }
                }
            }

            GridLayout {
                width: parent.width; columns: width >= 760 ? 2 : 1; columnSpacing: 12; rowSpacing: 12
                Rectangle {
                    Layout.fillWidth: true; Layout.preferredHeight: 340; radius: 14; color: root.card; border.color: root.edge
                    ColumnLayout {
                        anchors.fill: parent; anchors.margins: 14; spacing: 6
                        RowLayout {
                            Layout.fillWidth: true
                            Text { text: "MAGNETOMETER CALIBRATION"; color: root.cyan; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1; Layout.fillWidth: true }
                            ComboBox {
                                model: ["Dryland", "Tethered water"]
                                currentIndex: String(backendState.navigationTestEnvironment || "dry") === "water" ? 1 : 0
                                onActivated: function(index) { if (backendObject) backendObject.setNavigationTestEnvironment(index === 0 ? "dry" : "water") }
                            }
                        }
                        Text { Layout.fillWidth: true; wrapMode: Text.WordWrap; text: "Base calibration (motors off): rotate the level buoy through full turns, then save and upload the hard/soft-iron calibration."; color: root.muted; font.pixelSize: 10 }
                        Flow {
                            Layout.fillWidth: true; spacing: 5
                            ActionButton { text: backendState.magnetometerCalibrationActive ? "Capturing…" : "Start base"; primary: true; enabled: !backendState.magnetometerCalibrationActive && !backendState.testSessionActive; onClicked: if (backendObject) backendObject.startMagnetometerCalibration() }
                            ActionButton { text: "Finish & save"; enabled: Boolean(backendState.magnetometerCalibrationActive) && Number(backendState.magnetometerCalibrationSamples || 0) >= 20; onClicked: if (backendObject) backendObject.finishMagnetometerCalibration() }
                            ActionButton { text: "Upload base"; enabled: Boolean(backendState.magnetometerCalibrationSaved) && root.connected(); onClicked: if (backendObject) backendObject.uploadMagnetometerCalibration() }
                            ActionButton { text: "Cancel"; enabled: Boolean(backendState.magnetometerCalibrationActive); onClicked: if (backendObject) backendObject.cancelMagnetometerCalibration() }
                        }
                        Text { Layout.fillWidth: true; text: (backendState.magnetometerCalibrationStatus || "No saved base calibration") + (backendState.magnetometerCalibrationActive ? " · " + Number(backendState.magnetometerCalibrationSamples || 0) + " samples" : "") + " · Buoy: " + (backendState.magnetometerBuoyConfigStatus || "not checked"); color: root.muted; font.pixelSize: 9; elide: Text.ElideRight }
                        Rectangle { Layout.fillWidth: true; height: 1; color: root.edge }
                        RowLayout {
                            Layout.fillWidth: true
                            Text { text: "MOTOR CORRECTION · WATER ONLY"; color: root.amber; font.pixelSize: 10; font.bold: true; Layout.fillWidth: true }
                            ComboBox {
                                model: ["Motors-off baseline", "Motor load"]
                                id: compassCaptureType
                            }
                            ComboBox { id: compassMotor; model: ["Rear", "Front left", "Front right"]; enabled: compassCaptureType.currentIndex === 1 }
                            ComboBox { id: compassDirection; model: ["Forward", "Reverse"]; enabled: compassCaptureType.currentIndex === 1 }
                            SpinBox { id: compassPower; from: 10; to: 100; stepSize: 10; value: 20; editable: true; enabled: compassCaptureType.currentIndex === 1; Layout.preferredWidth: 88 }
                        }
                        Text { Layout.fillWidth: true; wrapMode: Text.WordWrap; text: "Keep the buoy at the same heading and as still as possible. Capture a motors-off baseline, then use the transmitter to run one motor at a time at the selected load while recording. The panel logs actual motor outputs, current, raw field and gyro; it does not command the motors."; color: root.muted; font.pixelSize: 9 }
                        RowLayout {
                            Layout.fillWidth: true; spacing: 6
                            ActionButton {
                                text: backendState.testSessionActive ? "Capture running…" : (compassCaptureType.currentIndex === 0 ? "Capture baseline" : "Capture motor load")
                                primary: true
                                enabled: root.connected() && backendState.imuStreamEnabled && backendState.navigationTestEnvironment === "water" && !backendState.testSessionActive
                                onClicked: if (backendObject) backendObject.startCompassLoadCapture(compassCaptureType.currentIndex === 0 ? "motors_off_baseline" : "motor_load", compassCaptureType.currentIndex === 0 ? "none" : ["rear", "front_left", "front_right"][compassMotor.currentIndex], compassCaptureType.currentIndex === 0 ? "none" : (compassDirection.currentIndex === 0 ? "forward" : "reverse"), compassCaptureType.currentIndex === 0 ? 0 : compassPower.value)
                            }
                            ActionButton { text: "Stop & save"; enabled: backendState.testSessionActive && Boolean(backendState.compassLoadCapturePhase); onClicked: if (backendObject) backendObject.stopTestSession() }
                            Text { Layout.fillWidth: true; text: backendState.compassLoadCaptureStatus || "Start IMU stream and select water"; color: root.muted; font.pixelSize: 9; elide: Text.ElideRight }
                        }
                        Text { text: "Field " + (backendState.magnetometerRaw || "waiting") + "   ·   Current " + (backendState.currentDraw || "—") + "   ·   Gyro Z " + Number(backendState.navigationMeasuredYawDps || 0).toFixed(1) + " °/s"; color: root.ink; font.pixelSize: 9; elide: Text.ElideRight; Layout.fillWidth: true }
                        Text { text: backendState.compassLoadBaselineStatus || "No water baseline saved"; color: root.muted; font.pixelSize: 9; elide: Text.ElideRight; Layout.fillWidth: true }
                        Text {
                            Layout.fillWidth: true; elide: Text.ElideRight
                            text: backendState.compassLoadLastSummary && backendState.compassLoadLastSummary.delta_from_baseline
                                ? "Last load Δ raw field   X " + Number(backendState.compassLoadLastSummary.delta_from_baseline.mag_x).toFixed(1) + "   Y " + Number(backendState.compassLoadLastSummary.delta_from_baseline.mag_y).toFixed(1) + "   Z " + Number(backendState.compassLoadLastSummary.delta_from_baseline.mag_z).toFixed(1) + "   Heading " + Number(backendState.compassLoadLastSummary.delta_from_baseline.heading_deg || 0).toFixed(1) + "°"
                                : "Last load capture will show its field change here after a matching baseline"
                            color: root.blue; font.pixelSize: 9
                        }
                    }
                }
            }
        }
    }
}
