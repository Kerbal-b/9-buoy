import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Item {
    id: root

    property var backendObject
    property var backendState: ({})

    function clamp(value, minimum, maximum) {
        return Math.max(minimum, Math.min(maximum, value))
    }

    function connected() {
        return String(backendState.serialStatus || "").indexOf("Connected") === 0
    }

    function signed(value, digits) {
        const number = Number(value || 0)
        return (number >= 0 ? "+" : "") + number.toFixed(digits)
    }

    onBackendStateChanged: vectorCanvas.requestPaint()

    ScrollView {
        id: navigationScroll
        anchors.fill: parent
        clip: true

        Column {
            x: 18
            y: 16
            width: navigationScroll.availableWidth - 36
            spacing: 14

            Text {
                text: "Navigation Test"
                color: "#e6ecf2"
                font.pixelSize: 22
                font.bold: true
            }

            Text {
                width: parent.width
                wrapMode: Text.WordWrap
                text: "Compare the controller's requested movement direction with MPU6050 linear acceleration. The panel removes slowly changing gravity and hull tilt before evaluating propulsion response. Assist applies a bounded direction correction while leaving throttle magnitude under operator control."
                color: "#9eb5c7"
                font.pixelSize: 13
            }

            Rectangle {
                width: parent.width
                height: 88
                radius: 10
                color: backendState.testSessionActive ? "#102d26" : "#121c28"
                border.color: backendState.testSessionActive ? "#3ddc97" : "#3b5064"

                Row {
                    anchors.fill: parent
                    anchors.margins: 12
                    spacing: 12

                    Rectangle {
                        width: 10
                        height: 10
                        radius: 5
                        anchors.verticalCenter: parent.verticalCenter
                        color: backendState.testSessionActive ? "#ff5c6c" : "#60788d"
                    }

                    Column {
                        width: parent.width - 278
                        anchors.verticalCenter: parent.verticalCenter
                        spacing: 4

                        Text {
                            text: backendState.testSessionActive
                                ? (backendState.navigationTestEnvironment === "water" ? "WATER IMU CALIBRATION" : "DRY-LAND BASELINE")
                                    + "  •  " + Number(backendState.testSessionSampleCount || 0) + " samples"
                                : (backendState.testSessionStatus || "Not recording")
                            color: backendState.testSessionActive ? "#8ce7c1" : "#dbe7ef"
                            font.pixelSize: 13
                            font.bold: true
                        }

                        Text {
                            width: parent.width
                            text: backendState.testSessionActive
                                ? "IMU " + Number(backendState.testSessionImuRateHz || 0).toFixed(1)
                                    + " Hz  •  " + Number(backendState.testSessionImuLossPercent || 0).toFixed(1)
                                    + "% timing loss  •  synchronized motor and power"
                                : (backendState.navigationTestEnvironment === "water"
                                    ? "In-water calibration: wall contacts remain in the recorded IMU signal"
                                    : "Dry baseline: select IN WATER below before the bathtub run")
                            color: "#8fa6b8"
                            font.pixelSize: 11
                            elide: Text.ElideRight
                        }

                        Text {
                            width: parent.width
                            visible: String(backendState.testSessionFile || "").length > 0
                            text: backendState.testSessionFile || ""
                            color: "#607f93"
                            font.pixelSize: 9
                            elide: Text.ElideMiddle
                        }
                    }

                    Button {
                        id: startRecordingButton
                        width: 104
                        height: 36
                        anchors.verticalCenter: parent.verticalCenter
                        text: backendState.navigationTestEnvironment === "water" ? "Start Water Test" : "Start Dry Test"
                        enabled: connected() && !Boolean(backendState.testSessionActive)
                        onClicked: if (backendObject) backendObject.startTestSession()

                        background: Rectangle {
                            radius: 8
                            color: startRecordingButton.enabled ? "#1f604d" : "#25313c"
                            border.color: startRecordingButton.enabled ? "#3ddc97" : "#465463"
                        }
                        contentItem: Text {
                            text: startRecordingButton.text
                            color: startRecordingButton.enabled ? "#ecfff7" : "#6f7e8a"
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            font.pixelSize: 12
                            font.bold: true
                        }
                    }

                    Button {
                        id: stopRecordingButton
                        width: 104
                        height: 36
                        anchors.verticalCenter: parent.verticalCenter
                        text: "Stop & Save"
                        enabled: Boolean(backendState.testSessionActive)
                        onClicked: if (backendObject) backendObject.stopTestSession()

                        background: Rectangle {
                            radius: 8
                            color: stopRecordingButton.enabled ? "#67313a" : "#25313c"
                            border.color: stopRecordingButton.enabled ? "#ff7180" : "#465463"
                        }
                        contentItem: Text {
                            text: stopRecordingButton.text
                            color: stopRecordingButton.enabled ? "#fff0f2" : "#6f7e8a"
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            font.pixelSize: 12
                            font.bold: true
                        }
                    }
                }
            }

            Rectangle {
                width: parent.width
                height: 64
                radius: 10
                color: backendState.navigationAssistActive ? "#17382f" : "#121c28"
                border.color: backendState.navigationAssistActive ? "#57d4a5" : "#3b5064"

                Row {
                    anchors.fill: parent
                    anchors.margins: 12
                    spacing: 12

                    Rectangle {
                        width: 12
                        height: 12
                        radius: 6
                        anchors.verticalCenter: parent.verticalCenter
                        color: backendState.navigationAssistActive
                            ? "#57d4a5"
                            : (backendState.navigationImuReady ? "#50aaff" : "#ffaa46")
                    }

                    Column {
                        width: parent.width - 190
                        anchors.verticalCenter: parent.verticalCenter
                        spacing: 3
                        Text {
                            text: backendState.navigationStatus || "Waiting"
                            color: "#e6ecf2"
                            font.pixelSize: 14
                            font.bold: true
                        }
                        Text {
                            text: connected() ? "Buoy connected" : "Buoy disconnected"
                            color: connected() ? "#79d3a6" : "#e59a67"
                            font.pixelSize: 11
                        }
                    }

                    Switch {
                        id: assistSwitch
                        width: 140
                        text: "Balance assist"
                        checked: Boolean(backendState.navigationAssistEnabled)
                        enabled: connected() && Boolean(backendState.navigationImuReady)
                        onToggled: if (backendObject) backendObject.setNavigationAssist(checked)
                    }
                }
            }

            Rectangle {
                width: parent.width
                height: 88
                radius: 10
                color: "#121c28"
                border.color: "#3b5064"

                Column {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 5

                    Row {
                        width: parent.width
                        spacing: 10
                        Text {
                            width: 132
                            text: "Mixer Power Limit"
                            color: "#e6ecf2"
                            font.pixelSize: 13
                            font.bold: true
                        }
                        Slider {
                            width: parent.width - 202
                            height: 22
                            from: 10
                            to: 100
                            stepSize: 5
                            value: Number(backendState.mixerPowerPercent || 50)
                            onMoved: if (backendObject) backendObject.setMixerPowerLimit(value)
                        }
                        Text {
                            width: 50
                            text: Number(backendState.mixerPowerPercent || 50) + "%"
                            color: "#50aaff"
                            font.pixelSize: 13
                            font.bold: true
                            horizontalAlignment: Text.AlignRight
                        }
                    }

                    Text {
                        width: parent.width
                        wrapMode: Text.WordWrap
                        text: "Caps the final translation + yaw mix for every control mode, before each motor's configured PWM limit."
                        color: "#8fa6b8"
                        font.pixelSize: 11
                    }
                }
            }

            Rectangle {
                width: parent.width
                height: 88
                radius: 10
                color: backendState.keyboardDriveEnabled ? "#172d3d" : "#121c28"
                border.color: backendState.keyboardDriveEnabled ? "#50aaff" : "#3b5064"

                Row {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 14

                    Switch {
                        id: keyboardDriveSwitch
                        width: 170
                        text: "Keyboard Drive"
                        checked: Boolean(backendState.keyboardDriveEnabled)
                        enabled: connected()
                        onClicked: if (backendObject) backendObject.setKeyboardDriveEnabled(checked)
                    }

                    Column {
                        width: parent.width - 184
                        anchors.verticalCenter: parent.verticalCenter
                        spacing: 3
                        Text {
                            text: backendState.keyboardInputSummary || "Idle"
                            color: "#e6ecf2"
                            font.pixelSize: 13
                            font.bold: true
                        }
                        Text {
                            text: "W/S forward-reverse, A/D left-right, Left/Right arrows yaw. Space stops and requires release before motion can resume."
                            color: "#8fa6b8"
                            font.pixelSize: 11
                        }
                    }
                }
            }

            MovementInstrumentPanel {
                width: parent.width
                height: 340
                backendState: root.backendState
                backendObject: root.backendObject
            }

            Rectangle {
                width: parent.width
                height: 390
                radius: 12
                color: "#0b131d"
                border.color: "#33485d"

                Canvas {
                    id: vectorCanvas
                    anchors.fill: parent
                    anchors.margins: 12

                    onWidthChanged: requestPaint()
                    onHeightChanged: requestPaint()

                    onPaint: {
                        const ctx = getContext("2d")
                        ctx.reset()
                        ctx.clearRect(0, 0, width, height)
                        if (width <= 1 || height <= 1) return

                        const cx = width / 2
                        const cy = height / 2 + 12
                        const radius = Math.max(1, Math.min(width, height) * 0.36)

                        ctx.strokeStyle = "#294054"
                        ctx.lineWidth = 1
                        ctx.beginPath()
                        ctx.arc(cx, cy, radius, 0, Math.PI * 2)
                        ctx.moveTo(cx - radius, cy)
                        ctx.lineTo(cx + radius, cy)
                        ctx.moveTo(cx, cy - radius)
                        ctx.lineTo(cx, cy + radius)
                        ctx.stroke()

                        ctx.fillStyle = "#71889b"
                        ctx.font = "11px sans-serif"
                        ctx.textAlign = "center"
                        ctx.fillText("FORWARD", cx, cy - radius - 10)
                        ctx.fillText("REVERSE", cx, cy + radius + 18)
                        ctx.textAlign = "left"
                        ctx.fillText("RIGHT", cx + radius + 8, cy + 4)
                        ctx.textAlign = "right"
                        ctx.fillText("LEFT", cx - radius - 8, cy + 4)

                        function arrow(x, y, color, widthValue, label) {
                            const endX = cx + clamp(x, -1, 1) * radius
                            const endY = cy - clamp(y, -1, 1) * radius
                            const angle = Math.atan2(endY - cy, endX - cx)
                            const head = 11
                            ctx.strokeStyle = color
                            ctx.fillStyle = color
                            ctx.lineWidth = widthValue
                            ctx.beginPath()
                            ctx.moveTo(cx, cy)
                            ctx.lineTo(endX, endY)
                            ctx.stroke()
                            ctx.beginPath()
                            ctx.moveTo(endX, endY)
                            ctx.lineTo(endX - head * Math.cos(angle - 0.45), endY - head * Math.sin(angle - 0.45))
                            ctx.lineTo(endX - head * Math.cos(angle + 0.45), endY - head * Math.sin(angle + 0.45))
                            ctx.closePath()
                            ctx.fill()
                            ctx.font = "12px sans-serif"
                            ctx.textAlign = "left"
                            ctx.fillText(label, endX + 8, endY - 7)
                        }

                        const requestedX = Number(backendState.navigationRequestedTurn || 0) / 100.0
                        const requestedY = Number(backendState.navigationRequestedThrust || 0) / 100.0
                        const correctedX = Number(backendState.navigationCorrectedTurn || 0) / 100.0
                        const correctedY = Number(backendState.navigationCorrectedThrust || 0) / 100.0
                        const measuredX = Number(backendState.navigationMeasuredXG || 0)
                        const measuredY = Number(backendState.navigationMeasuredYG || 0)
                        const measuredMagnitude = Math.sqrt(measuredX * measuredX + measuredY * measuredY)
                        const measuredScale = clamp(measuredMagnitude / 0.15, 0, 1)
                        const measuredDrawX = measuredMagnitude > 0.0001 ? measuredX / measuredMagnitude * measuredScale : 0
                        const measuredDrawY = measuredMagnitude > 0.0001 ? measuredY / measuredMagnitude * measuredScale : 0

                        arrow(requestedX, requestedY, "#50aaff", 5, "Requested")
                        arrow(measuredDrawX, measuredDrawY, "#ffaa46", 4, "Measured")
                        arrow(correctedX, correctedY, "#57d4a5", 3, "Corrected")

                        const yaw = Number(backendState.navigationRequestedYaw || 0) / 100.0
                        if (Math.abs(yaw) > 0.001) {
                            const yawRadius = radius + 20
                            const startAngle = -Math.PI / 2
                            const endAngle = startAngle + yaw * Math.PI * 1.5
                            ctx.strokeStyle = "#c58cff"
                            ctx.lineWidth = 4
                            ctx.beginPath()
                            ctx.arc(cx, cy, yawRadius, startAngle, endAngle, yaw < 0)
                            ctx.stroke()
                            ctx.fillStyle = "#c58cff"
                            ctx.textAlign = "center"
                            ctx.fillText("Yaw " + signed(backendState.navigationRequestedYaw, 0) + "%", cx, cy + yawRadius + 20)
                        }

                        ctx.fillStyle = "#dce6ee"
                        ctx.beginPath()
                        ctx.arc(cx, cy, 5, 0, Math.PI * 2)
                        ctx.fill()
                    }
                }

                Row {
                    x: 16
                    y: 12
                    spacing: 18
                    Repeater {
                        model: [
                            { label: "Requested", color: "#50aaff" },
                            { label: "Measured linear acceleration", color: "#ffaa46" },
                            { label: "Corrected command", color: "#57d4a5" },
                            { label: "Yaw", color: "#c58cff" }
                        ]
                        delegate: Row {
                            spacing: 5
                            Rectangle { width: 14; height: 4; color: modelData.color; anchors.verticalCenter: parent.verticalCenter }
                            Text { text: modelData.label; color: "#aebdca"; font.pixelSize: 11 }
                        }
                    }
                }
            }

            GridLayout {
                width: parent.width
                columns: 4
                columnSpacing: 10
                rowSpacing: 10

                Repeater {
                    model: [
                        { label: "REQUESTED X / Y / YAW", value: signed(backendState.navigationRequestedTurn, 0) + ", " + signed(backendState.navigationRequestedThrust, 0) + ", " + signed(backendState.navigationRequestedYaw, 0) + "%" },
                        { label: "LINEAR ACCEL X / Y", value: signed(backendState.navigationMeasuredXG, 3) + ", " + signed(backendState.navigationMeasuredYG, 3) + " g" },
                        { label: "MEASURED YAW RATE", value: signed(backendState.navigationMeasuredYawDps, 1) + " dps" },
                        { label: "ERROR / CORRECTION", value: signed(backendState.navigationDirectionErrorDeg, 1) + " / " + signed(backendState.navigationCorrectionDeg, 1) + " deg" }
                    ]
                    delegate: Rectangle {
                        Layout.fillWidth: true
                        height: 68
                        radius: 9
                        color: "#121c28"
                        border.color: "#33485d"
                        Column {
                            anchors.fill: parent
                            anchors.margins: 10
                            spacing: 5
                            Text { text: modelData.label; color: "#7890a3"; font.pixelSize: 10; font.bold: true }
                            Text { text: modelData.value; color: "#e6ecf2"; font.pixelSize: 16; font.bold: true }
                        }
                    }
                }
            }

            Rectangle {
                width: parent.width
                height: tuningColumn.implicitHeight + 28
                radius: 12
                color: "#111923"
                border.color: "#33485d"

                Column {
                    id: tuningColumn
                    x: 14
                    y: 14
                    width: parent.width - 28
                    spacing: 10

                    Text { text: "Test Setup & Safety Limits"; color: "#e6ecf2"; font.pixelSize: 17; font.bold: true }

                    Row {
                        width: parent.width
                        spacing: 12
                        Button {
                            width: 150
                            text: "Zero IMU at Rest"
                            enabled: connected()
                            onClicked: if (backendObject) backendObject.zeroNavigationImu()
                        }
                        Button {
                            width: 150
                            text: "STOP ALL"
                            enabled: connected()
                            onClicked: {
                                if (backendObject) {
                                    backendObject.setNavigationAssist(false)
                                    backendObject.stopAllMotors()
                                }
                            }
                        }
                        CheckBox {
                            id: invertX
                            text: "Invert IMU X"
                            checked: Boolean(backendState.navigationInvertX)
                            onToggled: if (backendObject) backendObject.setNavigationAxisSigns(checked, invertY.checked, swapXY.checked)
                        }
                        CheckBox {
                            id: invertY
                            text: "Invert IMU Y"
                            checked: Boolean(backendState.navigationInvertY)
                            onToggled: if (backendObject) backendObject.setNavigationAxisSigns(invertX.checked, checked, swapXY.checked)
                        }
                        CheckBox {
                            id: swapXY
                            text: "Swap IMU X/Y"
                            checked: Boolean(backendState.navigationSwapXY)
                            onToggled: if (backendObject) backendObject.setNavigationAxisSigns(invertX.checked, invertY.checked, checked)
                        }
                    }

                    Row {
                        width: parent.width
                        spacing: 12
                        Text { width: 120; text: "Correction gain"; color: "#aebdca"; font.pixelSize: 13 }
                        Slider {
                            width: parent.width - 230
                            from: 0
                            to: 100
                            stepSize: 5
                            value: Number(backendState.navigationGainPercent || 35)
                            onMoved: if (backendObject) backendObject.setNavigationGain(value)
                        }
                        Text { width: 80; text: Number(backendState.navigationGainPercent || 0) + "%"; color: "#dce6ee"; font.pixelSize: 13 }
                    }

                    Row {
                        width: parent.width
                        spacing: 12
                        Text { width: 120; text: "Max correction"; color: "#aebdca"; font.pixelSize: 13 }
                        Slider {
                            width: parent.width - 230
                            from: 0
                            to: 45
                            stepSize: 1
                            value: Number(backendState.navigationMaxCorrectionDeg || 20)
                            onMoved: if (backendObject) backendObject.setNavigationMaxCorrection(value)
                        }
                        Text { width: 80; text: Number(backendState.navigationMaxCorrectionDeg || 0) + " deg"; color: "#dce6ee"; font.pixelSize: 13 }
                    }
                }
            }

            Rectangle {
                width: parent.width
                height: motorColumn.implicitHeight + 28
                radius: 12
                color: "#111923"
                border.color: "#33485d"

                Column {
                    id: motorColumn
                    x: 14
                    y: 14
                    width: parent.width - 28
                    spacing: 8
                    Text { text: "Motor Mix: Requested vs Corrected"; color: "#e6ecf2"; font.pixelSize: 17; font.bold: true }
                    Repeater {
                        model: [
                            { name: "Rear", requested: backendState.navigationRequestedRearMotor, corrected: backendState.navigationCorrectedRearMotor },
                            { name: "Front Left", requested: backendState.navigationRequestedFrontLeftMotor, corrected: backendState.navigationCorrectedFrontLeftMotor },
                            { name: "Front Right", requested: backendState.navigationRequestedFrontRightMotor, corrected: backendState.navigationCorrectedFrontRightMotor }
                        ]
                        delegate: Row {
                            width: parent.width
                            height: 30
                            spacing: 10
                            Text { width: 110; text: modelData.name; color: "#aebdca"; font.pixelSize: 13 }
                            Text { width: 140; text: "Requested " + signed(modelData.requested, 0) + "%"; color: "#50aaff"; font.pixelSize: 13 }
                            Text { width: 140; text: "Corrected " + signed(modelData.corrected, 0) + "%"; color: "#57d4a5"; font.pixelSize: 13 }
                            Text {
                                text: "Delta " + signed(Number(modelData.corrected || 0) - Number(modelData.requested || 0), 0) + "%"
                                color: "#ffaa46"
                                font.pixelSize: 13
                            }
                        }
                    }
                }
            }

            Text {
                width: parent.width
                wrapMode: Text.WordWrap
                text: "Test procedure: secure a physical emergency-stop method, place the buoy stationary in the water, press Zero IMU, then use short low-power movement pulses and verify that the orange linear-acceleration arrow matches the requested direction. A hand tilt will briefly move the arrow but should decay as gravity is rejected. Enable Balance assist only after axis direction is verified. Corrections and IMU values are recorded in the communication log. GPS or water-speed feedback is still required for steady-course control."
                color: "#e6a76f"
                font.pixelSize: 12
            }

            Item { width: 1; height: 12 }
        }
    }
}
