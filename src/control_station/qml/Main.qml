import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

ApplicationWindow {
    id: window
    width: 1600
    height: 1000
    visible: true
    title: "Buoy Control Station"
    color: "#0c121c"

    readonly property var s: backend.state
    readonly property string buoyCanvasKey: [
        s.turn, s.thrust, s.yaw,
        s.frontLeftMotor, s.frontLeftMotorPwm,
        s.frontRightMotor, s.frontRightMotorPwm,
        s.rearMotor, s.rearMotorPwm,
        s.motorOutputTelemetry
    ].join("|")
    onBuoyCanvasKeyChanged: if (buoyCanvas) buoyCanvas.requestPaint()

    function clamp(v, minv, maxv) {
        return Math.max(minv, Math.min(maxv, v))
    }

    function keyName(key) {
        if (key === Qt.Key_Left) return "left"
        if (key === Qt.Key_Right) return "right"
        if (key === Qt.Key_Up) return "up"
        if (key === Qt.Key_Down) return "down"
        return ""
    }

    function dashboardTabIndex(tab) {
        if (tab === "communications") return 1
        if (tab === "power") return 2
        if (tab === "motors") return 3
        if (tab === "navigation") return 4
        if (tab === "instrument_debug") return 5
        if (tab === "sd_debug") return 6
        return 0
    }

    function dashboardTabForIndex(index) {
        if (index === 1) return "communications"
        if (index === 2) return "power"
        if (index === 3) return "motors"
        if (index === 4) return "navigation"
        if (index === 5) return "instrument_debug"
        if (index === 6) return "sd_debug"
        return "overview"
    }

    function numericPrefix(text) {
        if (text === null || text === undefined) {
            return NaN
        }
        const matches = String(text).match(/-?\d+(?:\.\d+)?/)
        return matches ? Number(matches[0]) : NaN
    }

    function batteryVoltageValue() {
        return numericPrefix(s.batteryStatus)
    }

    function currentDrawValue() {
        return numericPrefix(s.currentDraw)
    }

    function batteryVoltageText() {
        const value = batteryVoltageValue()
        return Number.isFinite(value) ? value.toFixed(1) + " V" : "N/A"
    }

    function batteryCurrentText() {
        const value = currentDrawValue()
        return Number.isFinite(value) ? Math.abs(value).toFixed(1) + " A" : "N/A"
    }

    function batteryIsDischarging() {
        const value = currentDrawValue()
        return Number.isFinite(value) && value < 0
    }

    function batteryPercentText() {
        const match = String(s.batteryStatus || "").match(/(\d+(?:\.\d+)?)\s*%/)
        return match ? Number(match[1]).toFixed(0) + "%" : "N/A"
    }

    function handleKey(event, pressed) {
        if (!Boolean(s.keyboardDriveEnabled)) {
            return false
        }
        const name = keyName(event.key)
        if (!name) {
            return false
        }
        backend.setKeyboardInput(name, pressed)
        event.accepted = true
        return true
    }

    Item {
        id: keyCatcher
        anchors.fill: parent
        focus: true
        Component.onCompleted: forceActiveFocus()

        Keys.onPressed: function(event) {
            handleKey(event, true)
        }

        Keys.onReleased: function(event) {
            handleKey(event, false)
        }
    }

    Rectangle {
        id: buoyPanel
        x: 36
        y: 28
        width: 470
        height: 400
        radius: 18
        color: "#182230"
        border.color: "#41556b"
        border.width: 1

        Text {
            x: 20
            y: 16
            text: "Buoy View"
            color: "#f4f8fb"
            font.pixelSize: 27
            font.bold: true
        }

        Text {
            x: 20
            y: 50
            text: "LATERAL  " + s.turn + "%    |    THRUST  " + s.thrust + "%    |    YAW  " + (s.yaw || 0) + "%"
            color: "#7f99ad"
            font.pixelSize: 11
            font.bold: true
        }

        Rectangle {
            x: parent.width - 116
            y: 18
            width: 96
            height: 24
            radius: 12
            color: "#132b2a"
            border.color: "#285c51"

            Rectangle {
                x: 10
                anchors.verticalCenter: parent.verticalCenter
                width: 6
                height: 6
                radius: 3
                color: "#3ddc97"
            }

            Text {
                x: 23
                anchors.verticalCenter: parent.verticalCenter
                text: "MOTOR MIX"
                color: "#8ce7c1"
                font.pixelSize: 9
                font.bold: true
            }
        }

        Canvas {
            id: buoyCanvas
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.leftMargin: 24
            anchors.rightMargin: 24
            anchors.topMargin: 82
            anchors.bottomMargin: 18
            Component.onCompleted: requestPaint()
            onWidthChanged: requestPaint()
            onHeightChanged: requestPaint()
            onPaint: {
                const ctx = getContext("2d")
                ctx.reset()
                ctx.clearRect(0, 0, width, height)

                const cx = width / 2
                const hullCy = height * 0.45
                const turn = clamp(s.turn / 100.0, -1, 1)
                const thrust = clamp(s.thrust / 100.0, -1, 1)
                const yaw = clamp((Number(s.yaw) || 0) / 100.0, -1, 1)
                const frontOffsetX = Math.min(100, width * 0.23)
                const frontOffsetY = 40
                const rearOffsetY = 95
                const frontLeft = [cx - frontOffsetX, hullCy - frontOffsetY]
                const frontRight = [cx + frontOffsetX, hullCy - frontOffsetY]
                const rear = [cx, hullCy + rearOffsetY]
                const ductLength = 76
                const ductWidth = 36

                function signedValue(value) {
                    const numericValue = Number(value) || 0
                    return (numericValue > 0 ? "+" : "") + Math.round(numericValue)
                }

                function usageColor(value) {
                    if (value > 0) return "#3ddc97"
                    if (value < 0) return "#ffaa46"
                    return "#60788d"
                }

                function roundedRect(x, y, rectWidth, rectHeight, radius) {
                    const r = Math.min(radius, rectWidth / 2, rectHeight / 2)
                    ctx.beginPath()
                    ctx.moveTo(x + r, y)
                    ctx.lineTo(x + rectWidth - r, y)
                    ctx.quadraticCurveTo(x + rectWidth, y, x + rectWidth, y + r)
                    ctx.lineTo(x + rectWidth, y + rectHeight - r)
                    ctx.quadraticCurveTo(x + rectWidth, y + rectHeight, x + rectWidth - r, y + rectHeight)
                    ctx.lineTo(x + r, y + rectHeight)
                    ctx.quadraticCurveTo(x, y + rectHeight, x, y + rectHeight - r)
                    ctx.lineTo(x, y + r)
                    ctx.quadraticCurveTo(x, y, x + r, y)
                    ctx.closePath()
                }

                function motorPower(value, pwm) {
                    const appliedPwm = Number(pwm)
                    if (Number.isFinite(appliedPwm) && appliedPwm >= 0)
                        return clamp(appliedPwm / 255.0, 0, 1)
                    return clamp(Math.abs(Number(value) || 0) / 100.0, 0, 1)
                }

                function drawPowerLink(startPoint, endPoint, value, pwm) {
                    const power = motorPower(value, pwm)
                    if (power <= 0.002) return
                    const color = usageColor(value)
                    ctx.save()
                    ctx.shadowColor = color
                    ctx.shadowBlur = 5 + 8 * power
                    ctx.strokeStyle = color
                    ctx.globalAlpha = 0.35 + 0.65 * power
                    ctx.lineWidth = 1.5 + 4.5 * power
                    ctx.lineCap = "round"
                    ctx.beginPath()
                    ctx.moveTo(startPoint[0], startPoint[1])
                    ctx.lineTo(endPoint[0], endPoint[1])
                    ctx.stroke()
                    ctx.restore()
                }

                function drawMotorDuct(point, forceX, forceY, value, pwm, label, labelX, labelY) {
                    const magnitude = clamp(Math.abs(Number(value) || 0) / 100.0, 0, 1)
                    const power = motorPower(value, pwm)
                    const color = usageColor(value)
                    // Motor axes are stored as force on the buoy. Propeller wash
                    // points opposite that force; canvas Y increases downward.
                    const waterX = -forceX
                    const waterY = forceY
                    const direction = value < 0 ? -1 : 1
                    const angle = Math.atan2(waterY, waterX)

                    ctx.save()
                    ctx.translate(point[0], point[1])
                    ctx.rotate(angle)
                    ctx.shadowColor = power <= 0.002 ? "transparent" : color
                    ctx.shadowBlur = power <= 0.002 ? 0 : 8 + 8 * power
                    ctx.beginPath()
                    roundedRect(-ductLength / 2, -ductWidth / 2, ductLength, ductWidth, 12)
                    const ductGradient = ctx.createLinearGradient(
                        -ductLength / 2, 0, ductLength / 2, 0
                    )
                    ductGradient.addColorStop(0, "#101b25")
                    ductGradient.addColorStop(0.22, "#385065")
                    ductGradient.addColorStop(0.5, "#162633")
                    ductGradient.addColorStop(0.78, "#385065")
                    ductGradient.addColorStop(1, "#101b25")
                    ctx.fillStyle = ductGradient
                    ctx.fill()
                    ctx.strokeStyle = "#385064"
                    ctx.lineWidth = 2
                    ctx.stroke()

                    ctx.fillStyle = "#0a121b"
                    roundedRect(-ductLength / 2 + 6, -5, ductLength - 12, 10, 5)
                    ctx.fill()
                    if (power > 0.002) {
                        const meterWidth = (ductLength - 12) * power
                        const meterX = direction > 0
                            ? -ductLength / 2 + 6
                            : ductLength / 2 - 6 - meterWidth
                        ctx.fillStyle = color
                        roundedRect(meterX, -4, meterWidth, 8, 4)
                        ctx.fill()
                    }

                    ctx.strokeStyle = "#7590a5"
                    ctx.lineWidth = 2
                    ctx.beginPath()
                    ctx.moveTo(-8, -ductWidth / 2 + 3)
                    ctx.lineTo(-8, ductWidth / 2 - 3)
                    ctx.moveTo(8, -ductWidth / 2 + 3)
                    ctx.lineTo(8, ductWidth / 2 - 3)
                    ctx.stroke()

                    if (magnitude > 0.005) {
                        const arrowStart = direction * (ductLength / 2 - 7)
                        const arrowEnd = direction * (ductLength / 2 + 17)
                        ctx.beginPath()
                        ctx.strokeStyle = color
                        ctx.lineWidth = 3
                        ctx.lineCap = "round"
                        ctx.moveTo(arrowStart, 0)
                        ctx.lineTo(arrowEnd, 0)
                        ctx.stroke()
                        ctx.lineCap = "butt"
                        ctx.fillStyle = color
                        ctx.beginPath()
                        ctx.moveTo(arrowEnd, 0)
                        ctx.lineTo(arrowEnd - direction * 9, -6)
                        ctx.lineTo(arrowEnd - direction * 9, 6)
                        ctx.closePath()
                        ctx.fill()
                    }
                    ctx.restore()

                    ctx.fillStyle = "#8ca3b5"
                    ctx.font = "bold 9px sans-serif"
                    ctx.textAlign = "center"
                    ctx.textBaseline = "alphabetic"
                    ctx.fillText(label, labelX, labelY)

                    const pwmNumber = Number(pwm)
                    const pwmText = Number.isFinite(pwmNumber) && pwmNumber >= 0 ? Math.round(pwmNumber) : "--"
                    ctx.fillStyle = value === 0 ? "#6f8799" : color
                    ctx.font = "bold 10px sans-serif"
                    ctx.fillText(
                        signedValue(value) + "%  |  PWM " + pwmText,
                        labelX,
                        labelY + 14
                    )
                }

                // Hull outline and motor arms follow the measured current layout:
                // front centers are approximately +/-97 mm by -42 mm, rear is +100 mm.
                const hullGradient = ctx.createRadialGradient(cx - 20, hullCy - 24, 5, cx, hullCy, 72)
                hullGradient.addColorStop(0, "#20364a")
                hullGradient.addColorStop(0.65, "#152534")
                hullGradient.addColorStop(1, "#0f1b27")
                ctx.beginPath()
                ctx.fillStyle = hullGradient
                ctx.arc(cx, hullCy, 68, 0, Math.PI * 2)
                ctx.fill()
                ctx.strokeStyle = "#3b5a70"
                ctx.lineWidth = 2
                ctx.stroke()

                ctx.beginPath()
                ctx.strokeStyle = "#263f52"
                ctx.lineWidth = 1
                ctx.arc(cx, hullCy, 58, 0, Math.PI * 2)
                ctx.stroke()

                ctx.strokeStyle = "#476176"
                ctx.lineWidth = 6
                ctx.lineCap = "round"
                ctx.beginPath()
                ctx.moveTo(cx - 34, hullCy - 45)
                ctx.lineTo(frontLeft[0] + 16, frontLeft[1] + 10)
                ctx.moveTo(cx + 34, hullCy - 45)
                ctx.lineTo(frontRight[0] - 16, frontRight[1] + 10)
                ctx.moveTo(cx, hullCy + 58)
                ctx.lineTo(rear[0], rear[1] - 22)
                ctx.stroke()
                ctx.lineCap = "butt"

                drawPowerLink([cx - 34, hullCy - 45], frontLeft,
                              s.frontLeftMotor || 0, s.frontLeftMotorPwm)
                drawPowerLink([cx + 34, hullCy - 45], frontRight,
                              s.frontRightMotor || 0, s.frontRightMotorPwm)
                drawPowerLink([cx, hullCy + 58], rear,
                              s.rearMotor || 0, s.rearMotorPwm)

                // Yaw is rotational, so visualize it independently from the
                // blue translation vector as a direction-aware arc on the hull.
                const yawMagnitude = Math.abs(yaw)
                const yawRadius = 51
                const yawStartAngle = -Math.PI * 0.75
                if (yawMagnitude > 0.005) {
                    const yawSweep = yawMagnitude * Math.PI * 1.5
                    const yawEndAngle = yawStartAngle + (yaw > 0 ? yawSweep : -yawSweep)
                    const yawColor = "#c58cff"

                    ctx.save()
                    ctx.shadowColor = yawColor
                    ctx.shadowBlur = 8
                    ctx.beginPath()
                    ctx.strokeStyle = yawColor
                    ctx.lineWidth = 5
                    ctx.lineCap = "round"
                    ctx.arc(cx, hullCy, yawRadius, yawStartAngle, yawEndAngle, yaw < 0)
                    ctx.stroke()
                    ctx.restore()

                    const yawEndX = cx + Math.cos(yawEndAngle) * yawRadius
                    const yawEndY = hullCy + Math.sin(yawEndAngle) * yawRadius
                    const tangentAngle = yawEndAngle + (yaw > 0 ? Math.PI / 2 : -Math.PI / 2)
                    ctx.fillStyle = yawColor
                    ctx.beginPath()
                    ctx.moveTo(yawEndX, yawEndY)
                    ctx.lineTo(
                        yawEndX - 10 * Math.cos(tangentAngle - 0.48),
                        yawEndY - 10 * Math.sin(tangentAngle - 0.48)
                    )
                    ctx.lineTo(
                        yawEndX - 10 * Math.cos(tangentAngle + 0.48),
                        yawEndY - 10 * Math.sin(tangentAngle + 0.48)
                    )
                    ctx.closePath()
                    ctx.fill()
                }

                // Requested movement vector remains centered on the hull.
                const vectorScale = 58
                const vectorEndX = cx + turn * vectorScale
                const vectorEndY = hullCy - thrust * vectorScale
                const vectorDx = vectorEndX - cx
                const vectorDy = vectorEndY - hullCy
                const vectorMagnitude = Math.sqrt(vectorDx * vectorDx + vectorDy * vectorDy)
                ctx.strokeStyle = "#55b7ff"
                ctx.lineWidth = 4
                if (vectorMagnitude > 2) {
                    ctx.beginPath()
                    ctx.moveTo(cx, hullCy)
                    ctx.lineTo(vectorEndX, vectorEndY)
                    ctx.stroke()

                    const angle = Math.atan2(vectorDy, vectorDx)
                    ctx.fillStyle = "#55b7ff"
                    ctx.beginPath()
                    ctx.moveTo(vectorEndX, vectorEndY)
                    ctx.lineTo(vectorEndX - 10 * Math.cos(angle - 0.5), vectorEndY - 10 * Math.sin(angle - 0.5))
                    ctx.lineTo(vectorEndX - 10 * Math.cos(angle + 0.5), vectorEndY - 10 * Math.sin(angle + 0.5))
                    ctx.closePath()
                    ctx.fill()
                }

                ctx.fillStyle = "#55b7ff"
                ctx.beginPath()
                ctx.arc(cx, hullCy, 5, 0, Math.PI * 2)
                ctx.fill()

                ctx.textAlign = "center"
                ctx.textBaseline = "middle"
                ctx.font = "bold 9px sans-serif"
                ctx.fillStyle = yawMagnitude > 0.005 ? "#d8b8ff" : "#60788d"
                ctx.fillText("YAW " + signedValue(Number(s.yaw) || 0) + "%", cx, hullCy + 41)

                drawMotorDuct(frontLeft, 0.5, 0.8660254,
                              s.frontLeftMotor || 0, s.frontLeftMotorPwm,
                              "FRONT LEFT", frontLeft[0], frontLeft[1] - 58)
                drawMotorDuct(frontRight, -0.5, 0.8660254,
                              s.frontRightMotor || 0, s.frontRightMotorPwm,
                              "FRONT RIGHT", frontRight[0], frontRight[1] - 58)
                drawMotorDuct(rear, 1.0, 0.0,
                              s.rearMotor || 0, s.rearMotorPwm,
                              "REAR / WATER LEFT", rear[0], rear[1] + 34)

                const rearPwm = Math.max(0, Math.round(Number(s.rearMotorPwm) || 0))
                const leftPwm = Math.max(0, Math.round(Number(s.frontLeftMotorPwm) || 0))
                const rightPwm = Math.max(0, Math.round(Number(s.frontRightMotorPwm) || 0))
                const activeMotors = (rearPwm > 0 ? 1 : 0) + (leftPwm > 0 ? 1 : 0) + (rightPwm > 0 ? 1 : 0)
                const totalPwm = rearPwm + leftPwm + rightPwm
                const measuredCurrent = currentDrawValue()

                ctx.font = "bold 8px sans-serif"
                ctx.textAlign = "left"
                ctx.textBaseline = "middle"
                ctx.fillStyle = "#71899b"
                let demandText = Boolean(s.motorOutputTelemetry)
                    ? activeMotors + " ACTIVE  |  ACTUAL PWM " + totalPwm
                    : "WAITING FOR MOTOR OUTPUT"
                if (Number.isFinite(measuredCurrent)) demandText += "  |  " + Math.abs(measuredCurrent).toFixed(1) + " A"
                ctx.fillText(demandText, 2, height - 5)

                ctx.textAlign = "right"
                ctx.fillStyle = "#60788d"
                ctx.fillText("GREEN +  /  ORANGE -  /  ARROWS SHOW WATER FLOW", width - 2, 10)
            }
        }

    }

    Rectangle {
        id: statusPanel
        x: 36
        y: 448
        width: 470
        height: parent.height - y
        radius: 18
        color: "#182230"
        border.color: "#41556b"
        border.width: 1

        Text {
            x: 20
            y: 16
            text: "Buoy Status"
            color: "#e6ecf2"
            font.pixelSize: 26
            font.bold: true
        }

        Column {
            x: 20
            y: 62
            width: parent.width - 40
            spacing: 6

        Rectangle {
            width: parent.width
            height: 54
            radius: 10
            color: "#0f1722"
            border.color: "#395166"

            Text {
                x: 12
                anchors.verticalCenter: parent.verticalCenter
                text: "Control"
                color: "#9eb5c7"
                font.pixelSize: 13
            }

            Row {
                x: 102
                y: 8
                width: parent.width - 112
                height: 38
                spacing: 6

                Repeater {
                    model: [
                        { key: "keyboard", label: "Keyboard" },
                        { key: "controller", label: "Controller" },
                        { key: "auto", label: "Auto" }
                    ]

                    delegate: Rectangle {
                        required property var modelData
                        width: (parent.width - 12) / 3
                        height: parent.height
                        radius: 8
                        color: s.controlInputMode === modelData.key ? "#1d5270" : "#172331"
                        border.color: s.controlInputMode === modelData.key ? "#50aaff" : "#31475a"
                        border.width: s.controlInputMode === modelData.key ? 2 : 1

                        MouseArea {
                            anchors.fill: parent
                            cursorShape: Qt.PointingHandCursor
                            onClicked: backend.setControlInputMode(modelData.key)
                        }

                        Text {
                            anchors.centerIn: parent
                            text: modelData.label
                            color: s.controlInputMode === modelData.key ? "#f4f8fb" : "#9eb5c7"
                            font.pixelSize: 12
                            font.bold: s.controlInputMode === modelData.key
                        }
                    }
                }
            }
        }

        Repeater {
                model: [
                    { label: "Connection", value: s.serialStatus },
                    { label: "Telemetry source", value: s.telemetrySource || "No live telemetry" },
                    { label: "Firmware", value: s.firmwareVersion || "Unknown" },
                    { label: "Battery", value: s.batteryStatus },
                    { label: "Location", value: s.currentLocation },
                    { label: "Target", value: s.targetLocation },
                    { label: "Hold", value: s.holdPosition ? "ON" : "OFF" },
                    { label: "Motion", value: s.turn + ", " + s.thrust + ", yaw " + (s.yaw || 0) },
                    { label: "Ack", value: s.ackVector || "N/A" }
                ]

                delegate: Rectangle {
                    width: parent.width
                    height: 36
                    color: "#0f1722"
                    radius: 8
                    border.color: "#395166"
                    border.width: 1

                    Text {
                        x: 12
                        y: 9
                        visible: modelData.label !== "Battery"
                        text: modelData.label
                        color: "#9eb5c7"
                        font.pixelSize: 13
                    }

                    Text {
                        x: parent.width - width - 12
                        y: 9
                        visible: modelData.label !== "Battery"
                        text: modelData.value
                        color: "#e6ecf2"
                        font.pixelSize: 13
                        elide: Text.ElideRight
                    }

                    Row {
                        visible: modelData.label === "Battery"
                        anchors.fill: parent
                        anchors.leftMargin: 12
                        anchors.rightMargin: 12
                        spacing: 5

                        Text {
                            width: 62
                            height: parent.height
                            verticalAlignment: Text.AlignVCenter
                            text: "Battery"
                            color: "#9eb5c7"
                            font.pixelSize: 13
                        }

                        Item {
                            width: Math.max(0, parent.width - 62 - 5 - 22 - 5 - 58 - 5 - 50 - 5 - 22 - 5 - 58)
                            height: 1
                        }

                        Rectangle {
                            width: 22
                            height: 22
                            anchors.verticalCenter: parent.verticalCenter
                            radius: 6
                            color: "#253b50"
                            border.color: "#50aaff"
                            border.width: 1

                            Text {
                                anchors.centerIn: parent
                                text: "V"
                                color: "#50aaff"
                                font.pixelSize: 12
                                font.bold: true
                            }
                        }

                        Text {
                            width: 58
                            height: parent.height
                            verticalAlignment: Text.AlignVCenter
                            horizontalAlignment: Text.AlignRight
                            text: batteryVoltageText()
                            color: "#e6ecf2"
                            font.pixelSize: 13
                        }

                        Text {
                            width: 50
                            height: parent.height
                            verticalAlignment: Text.AlignVCenter
                            horizontalAlignment: Text.AlignRight
                            text: batteryPercentText()
                            color: "#3ddc97"
                            font.pixelSize: 13
                            font.bold: true
                        }

                        Rectangle {
                            width: 22
                            height: 22
                            anchors.verticalCenter: parent.verticalCenter
                            radius: 6
                            color: batteryIsDischarging() ? "#4a3024" : "#263f35"
                            border.color: batteryIsDischarging() ? "#ffaa46" : "#3ddc97"
                            border.width: 1

                            Text {
                                anchors.centerIn: parent
                                text: "A"
                                color: batteryIsDischarging() ? "#ffaa46" : "#3ddc97"
                                font.pixelSize: 12
                                font.bold: true
                            }
                        }

                        Text {
                            width: 58
                            height: parent.height
                            verticalAlignment: Text.AlignVCenter
                            horizontalAlignment: Text.AlignRight
                            text: batteryCurrentText()
                            color: batteryIsDischarging() ? "#ffaa46" : "#e6ecf2"
                            font.pixelSize: 13
                        }

                    }
                }
        }
            }
    }

    Rectangle {
        id: dashboardPanel
        x: 532
        y: 28
        width: 1032
        height: parent.height - y
        z: 0
        radius: 18
        color: "#182230"
        border.color: "#41556b"
        border.width: 1

        Text {
            x: 20
            y: 16
            text: "Dashboard"
            color: "#e6ecf2"
            font.pixelSize: 26
            font.bold: true
        }

        TabBar {
            id: dashboardTabs
            x: 16
            y: 54
            width: parent.width - 32
            currentIndex: dashboardTabIndex(backend.dashboardTab)
            onCurrentIndexChanged: {
                const tab = dashboardTabForIndex(currentIndex)
                backend.setDashboardTab(tab)
                if (tab === "motors") {
                    motorTestPanel.activate()
                } else if (motorTestPanel.testRunning) {
                    motorTestPanel.deactivate()
                }
                if (tab === "sd_debug") backend.refreshSdFiles()
            }

            TabButton { text: "Overview"; font.pixelSize: 12 }
            TabButton { text: "Communications"; font.pixelSize: 12 }
            TabButton { text: "Power"; font.pixelSize: 12 }
            TabButton { text: "Motor Test"; font.pixelSize: 12 }
            TabButton { text: "Navigation"; font.pixelSize: 12 }
            TabButton { text: "Instrument Debugging"; font.pixelSize: 12 }
            TabButton { text: "SD Card"; font.pixelSize: 12 }
        }

        StackLayout {
            x: 16
            y: 96
            width: parent.width - 32
            height: parent.height - 112
            currentIndex: dashboardTabIndex(backend.dashboardTab)

            OverviewPanel {
                Layout.fillWidth: true
                Layout.fillHeight: true
                backendObject: backend
                backendState: s
            }

            CommunicationsPanel {
                Layout.fillWidth: true
                Layout.fillHeight: true
                backendObject: backend
                backendState: s
            }

            PowerPanel {
                Layout.fillWidth: true
                Layout.fillHeight: true
                backendState: s
            }

            MotorTestPanel {
                id: motorTestPanel
                Layout.fillWidth: true
                Layout.fillHeight: true
                backendObject: backend
                backendState: s
            }

            NavigationTestPanel {
                Layout.fillWidth: true
                Layout.fillHeight: true
                backendObject: backend
                backendState: s
            }

            InstrumentPanel {
                Layout.fillWidth: true
                Layout.fillHeight: true
                backendObject: backend
                backendState: s
            }

            SdCardDebugPanel {
                Layout.fillWidth: true
                Layout.fillHeight: true
                backendObject: backend
            }
        }
    }
}


