import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Item {
    id: root
    property var backendObject
    property var backendState: ({})
    readonly property string vectorCanvasKey: [
        backendState.navigationRequestedTurn, backendState.navigationRequestedThrust,
        backendState.navigationCorrectedTurn, backendState.navigationCorrectedThrust,
        backendState.navigationMeasuredXG, backendState.navigationMeasuredYG
    ].join("|")
    readonly property string compassCanvasKey: String(backendState.magnetometerTiltHeadingDeg) + "|" + String(backendState.magnetometerHeadingDeg)
    readonly property string gpsCanvasKey: JSON.stringify(backendState.navigationGpsTrack || [])

    function connected() {
        return String(backendState.serialStatus || "").indexOf("Connected") === 0
    }

    function coordinate(index) {
        const parts = String(backendState.currentLocation || "").split(",")
        if (parts.length < 2 || String(backendState.currentLocation).indexOf("Unknown") === 0)
            return "Waiting for GPS fix"
        const value = Number(parts[index].trim())
        return Number.isFinite(value) ? value.toFixed(6) + "°" : "Waiting for GPS fix"
    }

    function angle(value) {
        if (!backendState.imuAccel || backendState.imuAccel === "N/A")
            return "—"
        return Number(value || 0).toFixed(1) + "°"
    }

    function estimatedDirection() {
        const x = Number(backendState.navigationEstimatedVelocityXMps || 0)
        const y = Number(backendState.navigationEstimatedVelocityYMps || 0)
        if (Math.hypot(x, y) < 0.05)
            return "—"
        return ((Math.atan2(x, y) * 180 / Math.PI + 360) % 360).toFixed(1) + "° from bow"
    }

    function magnetometerCalibrationLabel() {
        const buoyStatus = String(backendState.magnetometerBuoyConfigStatus || "")
        if (buoyStatus.indexOf("loaded") >= 0 || buoyStatus.indexOf("applying") >= 0 || buoyStatus.indexOf("saved to buoy") >= 0)
            return "Buoy calibration active"
        return backendState.magnetometerCalibrationSaved ? "Station calibration active" : "Uncalibrated"
    }

    onVectorCanvasKeyChanged: vectorCanvas.requestPaint()
    onCompassCanvasKeyChanged: compassCanvas.requestPaint()
    onGpsCanvasKeyChanged: gpsCanvas.requestPaint()

    ScrollView {
        anchors.fill: parent
        clip: true
        Column {
            x: 20
            y: 18
            width: parent.width - 40
            spacing: 16

            RowLayout {
                width: parent.width
                Text {
                    text: "Navigation"
                    color: "#e6ecf2"
                    font.pixelSize: 23
                    font.bold: true
                    Layout.fillWidth: true
                }
                Button {
                    text: backendState.imuStreamEnabled ? "Stop IMU stream" : "Start IMU stream"
                    enabled: root.connected()
                    onClicked: if (backendObject) backendObject.setImuStreamEnabled(!backendState.imuStreamEnabled)
                }
            }

            Text {
                width: parent.width
                text: "Position, compass bearing, hull attitude, and motion."
                wrapMode: Text.WordWrap
                color: "#9eb5c7"
                font.pixelSize: 13
            }

            GridLayout {
                width: parent.width
                columns: width >= 760 ? 2 : 1
                columnSpacing: 14
                rowSpacing: 14

                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 246
                    radius: 12
                    color: "#121c28"
                    border.color: "#3b5064"
                    Column {
                        anchors.fill: parent
                        anchors.margins: 16
                        spacing: 9
                        Text { text: "GPS POSITION"; color: "#76bfff"; font.pixelSize: 12; font.bold: true }
                        Text { text: root.coordinate(0); color: "#e6ecf2"; font.pixelSize: 19; font.bold: true; elide: Text.ElideRight; width: parent.width }
                        Text { text: root.coordinate(1); color: "#e6ecf2"; font.pixelSize: 19; font.bold: true; elide: Text.ElideRight; width: parent.width }
                        Text { text: backendState.currentLocation && backendState.currentLocation !== "Unknown" ? (backendState.telemetrySource || "GPS fix") : "Waiting for GPS fix"; color: "#8fa6b8"; font.pixelSize: 12 }
                        Row {
                            spacing: 7
                            Text { text: "◉"; color: "#76bfff"; font.pixelSize: 18; anchors.verticalCenter: parent.verticalCenter }
                            Text {
                                text: backendState.gpsSatelliteCount === undefined || backendState.gpsSatelliteCount === null
                                    ? "Satellites  —" : "Satellites  " + Number(backendState.gpsSatelliteCount) + "  ·  " + (backendState.gpsHdop === undefined || backendState.gpsHdop === null ? "HDOP —" : "HDOP " + Number(backendState.gpsHdop).toFixed(1))
                                color: "#dce7ef"; font.pixelSize: 12; anchors.verticalCenter: parent.verticalCenter
                            }
                        }
                        Text { text: backendState.gpsQualityStatus || "Waiting for GPS data"; color: backendState.gpsFixValid && Number(backendState.gpsSatelliteCount || 0) >= Number(backendState.gpsMinimumSatellites || 6) ? "#67d8ad" : "#e7bd75"; font.pixelSize: 11; elide: Text.ElideRight; width: parent.width }
                        Row {
                            spacing: 7
                            Text { text: "Minimum satellites"; color: "#8fa6b8"; font.pixelSize: 11; anchors.verticalCenter: parent.verticalCenter }
                            SpinBox { from: 4; to: 12; value: Number(backendState.gpsMinimumSatellites || 6); onValueModified: if (backendObject) backendObject.setGpsMinimumSatellites(value) }
                        }
                    }
                }

                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 202
                    radius: 12
                    color: "#121c28"
                    border.color: "#3b5064"
                    Row {
                        anchors.fill: parent
                        anchors.margins: 16
                        spacing: 14
                        Canvas {
                            id: compassCanvas
                            width: 158
                            height: 158
                            anchors.verticalCenter: parent.verticalCenter
                            onPaint: {
                                const ctx = getContext("2d")
                                ctx.clearRect(0, 0, width, height)
                                const cx = width / 2, cy = height / 2, radius = Math.min(width, height) * 0.43
                                const heading = Number(backendState.magnetometerTiltHeadingDeg !== undefined && backendState.magnetometerTiltHeadingDeg !== null ? backendState.magnetometerTiltHeadingDeg : backendState.magnetometerHeadingDeg || 0)
                                ctx.fillStyle = "#09121c"
                                ctx.beginPath(); ctx.arc(cx, cy, radius + 9, 0, Math.PI * 2); ctx.fill()
                                ctx.strokeStyle = "#8298aa"; ctx.lineWidth = 2
                                ctx.beginPath(); ctx.arc(cx, cy, radius + 7, 0, Math.PI * 2); ctx.stroke()
                                ctx.save()
                                ctx.translate(cx, cy)
                                ctx.rotate(-heading * Math.PI / 180)
                                ctx.strokeStyle = "#71869a"
                                ctx.fillStyle = "#cbd7e1"
                                ctx.textAlign = "center"
                                ctx.textBaseline = "middle"
                                for (let degree = 0; degree < 360; degree += 5) {
                                    const radians = degree * Math.PI / 180 - Math.PI / 2
                                    const cardinal = degree % 90 === 0
                                    const numbered = degree % 30 === 0
                                    const tickLength = cardinal ? 13 : numbered ? 10 : degree % 10 === 0 ? 7 : 4
                                    ctx.lineWidth = cardinal ? 2 : 1
                                    ctx.beginPath()
                                    ctx.moveTo(Math.cos(radians) * (radius - tickLength), Math.sin(radians) * (radius - tickLength))
                                    ctx.lineTo(Math.cos(radians) * radius, Math.sin(radians) * radius)
                                    ctx.stroke()
                                    if (degree % 30 === 0) {
                                        const cardinalLabel = degree === 0 ? "N" : degree === 90 ? "E" : degree === 180 ? "S" : degree === 270 ? "W" : String(degree)
                                        ctx.fillStyle = degree === 0 ? "#ff6b70" : "#d5e0e9"
                                        ctx.font = cardinal ? "bold 12px sans-serif" : "10px sans-serif"
                                        const labelRadius = radius - 25
                                        ctx.fillText(cardinalLabel, Math.cos(radians) * labelRadius, Math.sin(radians) * labelRadius)
                                    }
                                }
                                ctx.restore()
                                // Fixed lubber line and red heading index at the top of the case.
                                ctx.fillStyle = "#ff626f"
                                ctx.beginPath(); ctx.moveTo(cx - 8, cy - radius - 8); ctx.lineTo(cx + 8, cy - radius - 8); ctx.lineTo(cx, cy - radius + 9); ctx.closePath(); ctx.fill()
                                ctx.fillStyle = "#e7edf2"
                                ctx.beginPath(); ctx.arc(cx, cy, 5, 0, Math.PI * 2); ctx.fill()
                            }
                        }
                        Column {
                            width: parent.width - 184
                            anchors.verticalCenter: parent.verticalCenter
                            spacing: 9
                            Text { text: "COMPASS"; color: "#c39bff"; font.pixelSize: 11; font.bold: true }
                            Text { text: backendState.magnetometerTiltHeadingDeg === undefined || backendState.magnetometerTiltHeadingDeg === null ? (backendState.magnetometerHeadingDeg === undefined || backendState.magnetometerHeadingDeg === null ? "---" : Number(backendState.magnetometerHeadingDeg).toFixed(0) + "°*") : Number(backendState.magnetometerTiltHeadingDeg).toFixed(0) + "°"; color: "#e6ecf2"; font.pixelSize: 32; font.bold: true }
                            Text { text: backendState.magnetometerTiltStatus || "Waiting for tilt data"; color: backendState.magnetometerTiltHeadingDeg === undefined || backendState.magnetometerTiltHeadingDeg === null ? "#e7bd75" : "#67d8ad"; font.pixelSize: 10; wrapMode: Text.WordWrap; width: parent.width }
                            Text { text: root.magnetometerCalibrationLabel(); color: root.magnetometerCalibrationLabel() === "Uncalibrated" ? "#e7bd75" : "#67d8ad"; font.pixelSize: 11 }
                            Text { text: backendState.magnetometerRaw || "X — | Y — | Z —"; color: "#8fa6b8"; font.pixelSize: 10; elide: Text.ElideRight; width: parent.width }
                        }
                    }
                }

            }

            MovementInstrumentPanel {
                width: parent.width
                height: 340
                showSpeedReadout: false
                backendObject: root.backendObject
                backendState: root.backendState
            }

            Rectangle {
                width: parent.width
                height: 82
                radius: 12
                color: "#121c28"
                border.color: "#3b5064"
                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 12
                    ColumnLayout {
                        Layout.fillWidth: true
                        Text { text: "WATERLINE LEVEL"; color: "#e6ecf2"; font.pixelSize: 12; font.bold: true }
                        Text { text: "Float normally, stop propulsion, then calibrate. " + (backendState.navigationLevelCalibrationStatus || ""); color: "#9eb5c7"; font.pixelSize: 11; elide: Text.ElideRight; Layout.fillWidth: true }
                    }
                    Button {
                        text: backendState.navigationLevelCalibrated ? "Recalibrate level" : "Calibrate level"
                        enabled: Boolean(backendState.imuAccel && backendState.imuAccel !== "N/A")
                        onClicked: if (backendObject) backendObject.calibrateNavigationLevel()
                    }
                }
            }

            Rectangle {
                width: parent.width
                height: 260
                radius: 12
                color: "#121c28"
                border.color: "#3b5064"
                Row {
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 16
                    Column {
                        width: parent.width * 0.58
                        spacing: 8
                        Text { text: "GPS MOVEMENT VECTOR"; color: "#76bfff"; font.pixelSize: 12; font.bold: true }
                        Canvas {
                            id: gpsCanvas
                            width: parent.width
                            height: 206
                            onPaint: {
                                const ctx = getContext("2d")
                                ctx.clearRect(0, 0, width, height)
                                const track = backendState.navigationGpsTrack || []
                                const current = track.length ? track[track.length - 1] : null
                                const cx = width / 2, cy = height / 2
                                ctx.strokeStyle = "#31465a"; ctx.lineWidth = 1
                                ctx.beginPath(); ctx.moveTo(cx, 8); ctx.lineTo(cx, height - 8); ctx.moveTo(8, cy); ctx.lineTo(width - 8, cy); ctx.stroke()
                                ctx.fillStyle = "#9eb5c7"; ctx.font = "10px sans-serif"; ctx.fillText("N", cx + 5, 13)
                                if (!current || track.length < 2) return
                                const latScale = 111320, lonScale = 111320 * Math.cos(Number(current.latitude) * Math.PI / 180)
                                const offsets = track.map(function(p) { return { x: (Number(p.longitude) - Number(current.longitude)) * lonScale, y: (Number(p.latitude) - Number(current.latitude)) * latScale } })
                                let max = 3
                                offsets.forEach(function(p) { max = Math.max(max, Math.abs(p.x), Math.abs(p.y)) })
                                const scale = Math.min(width, height) * 0.42 / max
                                ctx.strokeStyle = "#63b9ff"; ctx.lineWidth = 2; ctx.beginPath()
                                offsets.forEach(function(p, i) { const x = cx + p.x * scale, y = cy - p.y * scale; if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y) })
                                ctx.stroke()
                                ctx.fillStyle = "#69dfa9"; ctx.beginPath(); ctx.arc(cx, cy, 5, 0, Math.PI * 2); ctx.fill()
                                if (backendState.navigationGpsCourseAvailable) {
                                    const a = (Number(backendState.navigationGpsCourseDeg) - 90) * Math.PI / 180
                                    const ex = cx + Math.cos(a) * 35, ey = cy + Math.sin(a) * 35
                                    ctx.strokeStyle = "#ffbd66"; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(ex, ey); ctx.stroke()
                                }
                            }
                        }
                    }
                    Column {
                        width: parent.width * 0.42 - 16
                        anchors.verticalCenter: parent.verticalCenter
                        spacing: 10
                        Text { text: "GPS GROUND SPEED"; color: "#8fa6b8"; font.pixelSize: 10; font.bold: true }
                        Text { text: backendState.navigationGpsSpeedAvailable ? Number(backendState.navigationGpsSpeedMps || 0).toFixed(2) + " m/s" : "Waiting for GPS fixes"; color: "#76bfff"; font.pixelSize: 20; font.bold: true }
                        Text { text: backendState.navigationGpsCourseAvailable ? "GPS course  " + Number(backendState.navigationGpsCourseDeg || 0).toFixed(1) + "°" : "GPS course  —"; color: "#ffbd66"; font.pixelSize: 12 }
                        Text { text: backendState.magnetometerTiltHeadingDeg !== undefined && backendState.magnetometerTiltHeadingDeg !== null ? "Tilt compass  " + Number(backendState.magnetometerTiltHeadingDeg).toFixed(1) + "°" : backendState.magnetometerHeadingDeg === undefined || backendState.magnetometerHeadingDeg === null ? "Compass heading  —" : "Flat compass  " + Number(backendState.magnetometerHeadingDeg).toFixed(1) + "°"; color: "#c39bff"; font.pixelSize: 12 }
                        Text { text: backendState.navigationGpsCourseAvailable && (backendState.magnetometerTiltHeadingDeg !== undefined && backendState.magnetometerTiltHeadingDeg !== null || backendState.magnetometerHeadingDeg !== undefined && backendState.magnetometerHeadingDeg !== null) ? "Course vs compass  " + Number(backendState.navigationHeadingCourseOffsetDeg || 0).toFixed(1) + "°" : "Course comparison needs GPS motion"; color: "#e6ecf2"; font.pixelSize: 12; font.bold: true }
                        Text { width: parent.width; wrapMode: Text.WordWrap; text: "Offset can reflect current, wind, hull slip, compass alignment, or GPS noise; it cannot identify the cause by itself."; color: "#8fa6b8"; font.pixelSize: 10 }
                    }
                }
            }

            Rectangle {
                width: parent.width
                height: 174
                radius: 12
                color: "#121c28"
                border.color: "#3b5064"
                Column {
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 7
                    Text { text: "MAGNETOMETER CALIBRATION"; color: "#c39bff"; font.pixelSize: 12; font.bold: true }
                    Text {
                        width: parent.width
                        wrapMode: Text.WordWrap
                        text: "Place the buoy level, start calibration, then slowly rotate it through several complete turns. This learns hard-iron offsets and X/Y scale. It does not establish true north or compensate for tilt."
                        color: "#9eb5c7"
                        font.pixelSize: 12
                    }
                    Row {
                        width: parent.width
                        spacing: 8
                        Button {
                            text: backendState.magnetometerCalibrationActive ? "Capturing…" : "Start calibration"
                            enabled: !backendState.magnetometerCalibrationActive
                            onClicked: if (backendObject) backendObject.startMagnetometerCalibration()
                        }
                        Button {
                            text: "Finish & save"
                            enabled: Boolean(backendState.magnetometerCalibrationActive) && Number(backendState.magnetometerCalibrationSamples || 0) >= 20
                            onClicked: if (backendObject) backendObject.finishMagnetometerCalibration()
                        }
                        Button {
                            text: "Cancel"
                            enabled: Boolean(backendState.magnetometerCalibrationActive)
                            onClicked: if (backendObject) backendObject.cancelMagnetometerCalibration()
                        }
                        Button {
                            text: "Clear saved"
                            enabled: Boolean(backendState.magnetometerCalibrationSaved) && !backendState.magnetometerCalibrationActive
                            onClicked: if (backendObject) backendObject.resetMagnetometerCalibration()
                        }
                        Button {
                            text: "Upload to buoy"
                            enabled: Boolean(backendState.magnetometerCalibrationSaved) && root.connected()
                            onClicked: if (backendObject) backendObject.uploadMagnetometerCalibration()
                        }
                    }
                    Text {
                        width: parent.width
                        text: "Station: " + (backendState.magnetometerCalibrationStatus || "No saved calibration")
                            + (backendState.magnetometerCalibrationActive ? " · " + Number(backendState.magnetometerCalibrationSamples || 0) + " samples" : "")
                            + "   |   Buoy: " + (backendState.magnetometerBuoyConfigStatus || "Not checked")
                        color: "#8fa6b8"
                        font.pixelSize: 11
                        elide: Text.ElideRight
                    }
                }
            }

            Rectangle {
                width: parent.width
                height: 264
                radius: 12
                color: "#121c28"
                border.color: "#3b5064"
                Column {
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 10
                    Text { text: "MOVEMENT VECTOR & FORCES"; color: "#e6ecf2"; font.pixelSize: 13; font.bold: true }
                    Row {
                        width: parent.width
                        height: 190
                        spacing: 14
                        Canvas {
                            id: vectorCanvas
                            width: Math.min(280, parent.width * 0.45)
                            height: parent.height
                            onPaint: {
                                const ctx = getContext("2d")
                                ctx.clearRect(0, 0, width, height)
                                const cx = width / 2, cy = height / 2
                                ctx.strokeStyle = "#31465a"
                                ctx.lineWidth = 1
                                ctx.beginPath(); ctx.moveTo(12, cy); ctx.lineTo(width - 12, cy)
                                ctx.moveTo(cx, 12); ctx.lineTo(cx, height - 12); ctx.stroke()
                                function arrow(xValue, yValue, color, scale, fullScale, dash) {
                                    const x = Number(xValue || 0), y = Number(yValue || 0)
                                    const magnitude = Math.hypot(x, y)
                                    const length = Math.min(1, magnitude / fullScale) * scale
                                    const dx = magnitude > 0 ? (x / magnitude) * length : 0
                                    const dy = magnitude > 0 ? -(y / magnitude) * length : 0
                                    const ex = cx + dx, ey = cy + dy
                                    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2
                                    ctx.setLineDash(dash ? [5, 4] : [])
                                    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(ex, ey); ctx.stroke()
                                    const angle = Math.atan2(dy, dx), head = 8
                                    ctx.beginPath(); ctx.moveTo(ex, ey)
                                    ctx.lineTo(ex - head * Math.cos(angle - 0.5), ey - head * Math.sin(angle - 0.5))
                                    ctx.lineTo(ex - head * Math.cos(angle + 0.5), ey - head * Math.sin(angle + 0.5))
                                    ctx.closePath(); ctx.fill(); ctx.setLineDash([])
                                }
                                arrow(backendState.navigationRequestedTurn, backendState.navigationRequestedThrust, "#63b9ff", Math.min(width, height) * 0.38, 100, false)
                                arrow(backendState.navigationCorrectedTurn, backendState.navigationCorrectedThrust, "#ffb45f", Math.min(width, height) * 0.38, 100, false)
                                arrow(backendState.navigationMeasuredXG, backendState.navigationMeasuredYG, "#62dfa9", Math.min(width, height) * 0.38, 0.5, true)
                            }
                        }
                        GridLayout {
                            width: parent.width - vectorCanvas.width - 14
                            height: parent.height
                            columns: 2
                            rowSpacing: 8
                            columnSpacing: 12
                            Repeater {
                                model: [
                                    { label: "REQUESTED · lateral / thrust", value: Number(backendState.navigationRequestedTurn || 0).toFixed(0) + " / " + Number(backendState.navigationRequestedThrust || 0).toFixed(0) },
                                    { label: "REQUESTED · yaw", value: Number(backendState.navigationRequestedYaw || 0).toFixed(0) },
                                    { label: "CORRECTED · lateral / thrust", value: Number(backendState.navigationCorrectedTurn || 0).toFixed(0) + " / " + Number(backendState.navigationCorrectedThrust || 0).toFixed(0) },
                                    { label: "MEASURED · X / Y", value: Number(backendState.navigationMeasuredXG || 0).toFixed(3) + " / " + Number(backendState.navigationMeasuredYG || 0).toFixed(3) + " g" },
                                    { label: "ACCELERATION", value: Number(backendState.navigationMeasuredMagnitudeG || 0).toFixed(3) + " g" },
                                    { label: "ROLL / PITCH", value: root.angle(backendState.navigationRollDeg) + " / " + root.angle(backendState.navigationPitchDeg) },
                                    { label: "YAW RATE", value: Number(backendState.navigationMeasuredYawDps || 0).toFixed(1) + " deg/s" }
                                ]
                                delegate: Column {
                                    Layout.fillWidth: true
                                    spacing: 3
                                    Text { text: modelData.label; color: "#8fa6b8"; font.pixelSize: 9; elide: Text.ElideRight; width: parent.width }
                                    Text { text: modelData.value; color: "#e6ecf2"; font.pixelSize: 12; font.bold: true; elide: Text.ElideRight; width: parent.width }
                                }
                            }
                            Text {
                                Layout.columnSpan: 2
                                text: "Blue: requested   Orange: corrected   Dashed green: measured acceleration"
                                color: "#8fa6b8"
                                font.pixelSize: 10
                            }
                        }
                    }
                }
            }
        }
    }

}
