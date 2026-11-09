import QtQuick
import QtQuick.Layouts

    Rectangle {
        id: root
        property var backendState: ({})
        readonly property var s: backendState
    readonly property string buoyCanvasKey: [
        s.controlInputMode,
        s.turn, s.thrust, s.yaw,
        s.frontLeftMotor, s.frontLeftMotorPwm,
        s.frontRightMotor, s.frontRightMotorPwm,
        s.rearMotor, s.rearMotorPwm,
        s.rearMotorActualDrive, s.frontLeftMotorActualDrive, s.frontRightMotorActualDrive,
        s.motorOutputTelemetry, JSON.stringify(s.radioBuoyState || {})
    ].join("|")
    onBuoyCanvasKeyChanged: if (buoyCanvas) buoyCanvas.requestPaint()

    function clamp(v, minv, maxv) {
        return Math.max(minv, Math.min(maxv, v))
    }

    function hasObservedMotorOutput() {
        return Boolean(s.motorOutputTelemetry) || Boolean(s.radioBuoyState && s.radioBuoyState.motor_pwm)
    }

    function displayMotorDrive(index) {
        if (s.motorOutputTelemetry) {
            const values = [s.rearMotorActualDrive, s.frontLeftMotorActualDrive, s.frontRightMotorActualDrive]
            const value = Number(values[index]) || 0
            const pwm = Math.max(0, Number([s.rearMotorPwm, s.frontLeftMotorPwm, s.frontRightMotorPwm][index]) || 0)
            return value === 0 || pwm === 0 ? 0 : Math.sign(value) * Math.max(1, Math.round(pwm * 100 / 255))
        }
        if (s.radioBuoyState && s.radioBuoyState.motor_pwm) {
            const value = Number(s.radioBuoyState.motor_pwm[index]) || 0
            return value === 0 ? 0 : Math.sign(value) * Math.max(1, Math.round(Math.abs(value) * 100 / 255))
        }
        if (s.controlInputMode === "keyboard") {
            return Number([s.rearMotor, s.frontLeftMotor, s.frontRightMotor][index]) || 0
        }
        return 0
    }

    function displayMotorPwm(index) {
        if (s.motorOutputTelemetry) {
            return Number([s.rearMotorPwm, s.frontLeftMotorPwm, s.frontRightMotorPwm][index])
        }
        if (s.radioBuoyState && s.radioBuoyState.motor_pwm) {
            return Math.abs(Number(s.radioBuoyState.motor_pwm[index]) || 0)
        }
        return -1
    }

    function displayMotion() {
        if (!hasObservedMotorOutput()) {
            return { turn: Number(s.turn) || 0, thrust: Number(s.thrust) || 0, yaw: Number(s.yaw) || 0 }
        }
        // Invert the active three-motor mix to estimate the commanded motion
        // represented by the observed output. Saturation makes this approximate.
        const rear = displayMotorDrive(0) / 100
        const left = displayMotorDrive(1) / 100
        const right = displayMotorDrive(2) / 100
        const turn = (rear + (left - right) / 2) / 2
        const thrust = (left + right) / (8 * 0.8660254 / 3)
        const yaw = rear - 4 * turn / 3
        return {
            turn: Math.round(clamp(turn * 100, -100, 100)),
            thrust: Math.round(clamp(thrust * 100, -100, 100)),
            yaw: Math.round(clamp(yaw * 100, -100, 100))
        }
    }

    function currentDrawValue() {
        const match = String(s.currentDraw || "").match(/-?\d+(?:\.\d+)?/)
        return match ? Number(match[0]) : NaN
    }

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
            text: (root.hasObservedMotorOutput() ? "MOTOR OUTPUT EST.  "
                  : (s.controlInputMode === "tx12" ? "TX12 OUTPUT UNKNOWN  " : "KEYBOARD REQUEST  "))
                  + "LATERAL " + root.displayMotion().turn + "%  |  THRUST "
                  + root.displayMotion().thrust + "%  |  YAW " + root.displayMotion().yaw + "%"
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
                const drawingScale = Math.min(1.18, Math.max(0.65, width / 390),
                                              Math.max(0.65, height / 280))
                ctx.save()
                ctx.translate(width / 2, height * 0.45)
                ctx.scale(drawingScale, drawingScale)
                ctx.translate(-width / 2, -height * 0.45)

                const cx = width / 2
                const hullCy = height * 0.45
                const motion = root.displayMotion()
                const turn = clamp(motion.turn / 100.0, -1, 1)
                const thrust = clamp(motion.thrust / 100.0, -1, 1)
                const yaw = clamp(motion.yaw / 100.0, -1, 1)
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
                              root.displayMotorDrive(1), root.displayMotorPwm(1))
                drawPowerLink([cx + 34, hullCy - 45], frontRight,
                              root.displayMotorDrive(2), root.displayMotorPwm(2))
                drawPowerLink([cx, hullCy + 58], rear,
                              root.displayMotorDrive(0), root.displayMotorPwm(0))

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
                ctx.fillText("YAW " + signedValue(motion.yaw) + "%", cx, hullCy + 41)

                drawMotorDuct(frontLeft, 0.5, 0.8660254,
                              root.displayMotorDrive(1), root.displayMotorPwm(1),
                              "FRONT LEFT", frontLeft[0], frontLeft[1] - 58)
                drawMotorDuct(frontRight, -0.5, 0.8660254,
                              root.displayMotorDrive(2), root.displayMotorPwm(2),
                              "FRONT RIGHT", frontRight[0], frontRight[1] - 58)
                drawMotorDuct(rear, 1.0, 0.0,
                              root.displayMotorDrive(0), root.displayMotorPwm(0),
                              "REAR / WATER LEFT", rear[0], rear[1] + 34)
                ctx.restore()

                const rearPwm = Math.max(0, Math.round(root.displayMotorPwm(0) || 0))
                const leftPwm = Math.max(0, Math.round(root.displayMotorPwm(1) || 0))
                const rightPwm = Math.max(0, Math.round(root.displayMotorPwm(2) || 0))
                const activeMotors = (rearPwm > 0 ? 1 : 0) + (leftPwm > 0 ? 1 : 0) + (rightPwm > 0 ? 1 : 0)
                const totalPwm = rearPwm + leftPwm + rightPwm
                const measuredCurrent = currentDrawValue()

                ctx.font = "bold 8px sans-serif"
                ctx.textAlign = "left"
                ctx.textBaseline = "middle"
                ctx.fillStyle = "#71899b"
                let demandText = root.hasObservedMotorOutput()
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

