pragma ComponentBehavior: Bound

import QtQuick
import QtQuick.Layouts

Rectangle {
    id: root

    property var backendState: ({})
    property var backendObject

    function clamp(value, minimum, maximum) {
        return Math.max(minimum, Math.min(maximum, value))
    }

    function signed(value, digits) {
        const number = Number(value || 0)
        return (number >= 0 ? "+" : "") + number.toFixed(digits)
    }

    onBackendStateChanged: {
        attitudeCanvas.requestPaint()
        accelerationCanvas.requestPaint()
    }

    radius: 12
    color: "#09111a"
    border.color: "#3c5268"
    border.width: 1

    Text {
        x: 16
        y: 12
        text: "MOVEMENT INSTRUMENTS"
        color: "#e6ecf2"
        font.pixelSize: 14
        font.bold: true
        font.letterSpacing: 1.2
    }

    Text {
        x: 214
        y: 14
        text: "TEST ENVIRONMENT"
        color: "#7f96aa"
        font.pixelSize: 10
        font.bold: true
    }

    Row {
        x: 322
        y: 8
        spacing: 6

        Repeater {
            model: [
                { key: "dry", label: "DRY LAND" },
                { key: "water", label: "IN WATER" }
            ]

            delegate: Rectangle {
                id: environmentButton
                required property var modelData
                width: 76
                height: 26
                radius: 5
                color: root.backendState.navigationTestEnvironment === modelData.key ? "#21445d" : "#121d28"
                border.color: root.backendState.navigationTestEnvironment === modelData.key ? "#65bce9" : "#34495d"

                Text {
                    anchors.centerIn: parent
                    text: environmentButton.modelData.label
                    color: root.backendState.navigationTestEnvironment === environmentButton.modelData.key ? "#dff3ff" : "#829aad"
                    font.pixelSize: 9
                    font.bold: true
                }

                MouseArea {
                    anchors.fill: parent
                    cursorShape: Qt.PointingHandCursor
                    enabled: !Boolean(root.backendState.testSessionActive)
                    onClicked: if (root.backendObject) root.backendObject.setNavigationTestEnvironment(environmentButton.modelData.key)
                }
            }
        }
    }

    Text {
        anchors.right: parent.right
        anchors.rightMargin: 16
        y: 14
        text: root.backendState.navigationTestEnvironment === "water"
            ? "GPS preferred when a fresh fix is available"
            : "Water prediction requires a learned water profile"
        color: root.backendState.navigationTestEnvironment === "water" ? "#57d4a5" : "#e2a463"
        font.pixelSize: 10
    }

    RowLayout {
        x: 14
        y: 42
        width: parent.width - 28
        height: parent.height - 56
        spacing: 12

        Rectangle {
            Layout.preferredWidth: 190
            Layout.fillHeight: true
            radius: 9
            color: "#101b27"
            border.color: "#2f4559"

            Column {
                anchors.fill: parent
                anchors.margins: 14
                spacing: 7

                Text { text: "BEST AVAILABLE SPEED"; color: "#7f96aa"; font.pixelSize: 10; font.bold: true }
                Text {
                    text: Number(root.backendState.navigationBestSpeedMps || 0).toFixed(2)
                    color: "#72c7ff"
                    font.pixelSize: 42
                    font.bold: true
                }
                Text {
                    text: "m/s  ·  " + (root.backendState.navigationSpeedSource || "IMU estimate")
                    color: Boolean(root.backendState.navigationGpsSpeedAvailable) ? "#57d4a5" : "#e2a463"
                    font.pixelSize: 11
                }

                Rectangle { width: parent.width; height: 1; color: "#2a3c4d" }
                Text {
                    text: "Lateral  " + root.signed(root.backendState.navigationEstimatedVelocityXMps, 2) + " m/s"
                    color: "#d9e3eb"
                    font.pixelSize: 12
                }
                Text {
                    text: "Forward  " + root.signed(root.backendState.navigationEstimatedVelocityYMps, 2) + " m/s"
                    color: "#d9e3eb"
                    font.pixelSize: 12
                }
                Text {
                    text: Boolean(root.backendState.navigationGpsSpeedAvailable)
                        ? "GPS ground  " + Number(root.backendState.navigationGpsSpeedMps || 0).toFixed(2) + " m/s"
                        : "GPS ground  unavailable"
                    color: Boolean(root.backendState.navigationGpsSpeedAvailable) ? "#57d4a5" : "#7f96aa"
                    font.pixelSize: 11
                }

                Item { width: 1; height: 4 }
                Text { text: "ESTIMATE CONFIDENCE"; color: "#7f96aa"; font.pixelSize: 9; font.bold: true }
                Rectangle {
                    width: parent.width
                    height: 9
                    radius: 4
                    color: "#263746"
                    Rectangle {
                        width: parent.width * root.clamp(Number(root.backendState.navigationSpeedConfidencePercent || 0) / 100, 0, 1)
                        height: parent.height
                        radius: 4
                        color: Number(root.backendState.navigationSpeedConfidencePercent || 0) >= 50 ? "#57d4a5" : "#ffaa46"
                    }
                }
                Text {
                    text: Number(root.backendState.navigationSpeedConfidencePercent || 0) + "% · "
                        + (root.backendState.navigationTestEnvironment === "dry" ? "dry-test capped" : "decays without GPS")
                    color: "#9fb1c0"
                    font.pixelSize: 10
                }
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumWidth: 360
            radius: 9
            color: "#070c12"
            border.color: "#2f4559"

            Canvas {
                id: attitudeCanvas
                anchors.fill: parent
                anchors.margins: 8
                onWidthChanged: requestPaint()
                onHeightChanged: requestPaint()

                onPaint: {
                    const ctx = getContext("2d")
                    ctx.reset()
                    ctx.clearRect(0, 0, width, height)
                    if (width <= 0 || height <= 0)
                        return
                    const cx = width / 2
                    const cy = height / 2 + 4
                    const radius = Math.min(width, height) * 0.43
                    const roll = root.clamp(Number(root.backendState.navigationRollDeg || 0), -85, 85)
                    // Display convention: bow-up is positive and the external
                    // waterline moves down relative to the fixed hull symbol.
                    // The installed IMU reports the opposite pitch sign, so
                    // invert presentation only; navigation axes stay intact.
                    const pitch = root.clamp(-Number(root.backendState.navigationPitchDeg || 0), -45, 45)
                    const pitchOffset = pitch / 45 * radius * 0.75

                    ctx.save()
                    ctx.beginPath()
                    ctx.arc(cx, cy, radius, 0, Math.PI * 2)
                    ctx.clip()
                    ctx.translate(cx, cy)
                    ctx.rotate(-roll * Math.PI / 180)

                    // Marine attitude display: atmosphere above the horizon,
                    // open water below it, with the sensed waterline moving
                    // around a fixed hull reference.
                    ctx.fillStyle = "#142b3a"
                    ctx.fillRect(-radius * 2, -radius * 2, radius * 4, radius * 2 + pitchOffset)
                    ctx.fillStyle = "#087a9a"
                    ctx.fillRect(-radius * 2, pitchOffset, radius * 4, radius * 2)
                    ctx.strokeStyle = "#9ce7f2"
                    ctx.lineWidth = 3
                    ctx.beginPath()
                    ctx.moveTo(-radius * 2, pitchOffset)
                    ctx.lineTo(radius * 2, pitchOffset)
                    ctx.stroke()

                    ctx.strokeStyle = "#39afc8"
                    ctx.lineWidth = 1
                    for (let wave = 1; wave <= 4; ++wave) {
                        const waveY = pitchOffset + wave * 18
                        ctx.beginPath()
                        for (let waveX = -radius * 2; waveX <= radius * 2; waveX += 8) {
                            const y = waveY + Math.sin(waveX / 15 + wave) * 2
                            if (waveX === -radius * 2) ctx.moveTo(waveX, y)
                            else ctx.lineTo(waveX, y)
                        }
                        ctx.stroke()
                    }

                    ctx.strokeStyle = "#e4edf4"
                    ctx.fillStyle = "#e4edf4"
                    ctx.lineWidth = 1
                    ctx.font = "9px sans-serif"
                    ctx.textAlign = "center"
                    for (let mark = -30; mark <= 30; mark += 10) {
                        if (mark === 0) continue
                        const lineY = pitchOffset - mark / 45 * radius * 0.75
                        const halfWidth = mark % 20 === 0 ? 36 : 24
                        ctx.beginPath()
                        ctx.moveTo(-halfWidth, lineY)
                        ctx.lineTo(halfWidth, lineY)
                        ctx.stroke()
                        ctx.fillText(String(Math.abs(mark)), -halfWidth - 12, lineY + 3)
                        ctx.fillText(String(Math.abs(mark)), halfWidth + 12, lineY + 3)
                    }
                    ctx.restore()

                    ctx.strokeStyle = "#91a9bb"
                    ctx.lineWidth = 4
                    ctx.beginPath()
                    ctx.arc(cx, cy, radius, 0, Math.PI * 2)
                    ctx.stroke()

                    // Fixed top-down hull reference: port and starboard beams
                    // with a bow marker pointing forward.
                    ctx.strokeStyle = "#ffd05c"
                    ctx.fillStyle = "#ffd05c"
                    ctx.lineWidth = 3
                    ctx.beginPath()
                    ctx.moveTo(cx - 68, cy)
                    ctx.lineTo(cx - 22, cy)
                    ctx.moveTo(cx + 22, cy)
                    ctx.lineTo(cx + 68, cy)
                    ctx.stroke()
                    ctx.beginPath()
                    ctx.moveTo(cx, cy - 18)
                    ctx.lineTo(cx - 12, cy + 14)
                    ctx.lineTo(cx + 12, cy + 14)
                    ctx.closePath()
                    ctx.stroke()

                    ctx.fillStyle = "#dce7ef"
                    ctx.font = "bold 12px sans-serif"
                    ctx.textAlign = "center"
                    ctx.fillText("HULL ATTITUDE / WATERLINE", cx, 16)
                    ctx.font = "bold 9px sans-serif"
                    ctx.textAlign = "right"
                    ctx.fillText("PORT", cx - radius - 8, cy + 3)
                    ctx.textAlign = "left"
                    ctx.fillText("STBD", cx + radius + 8, cy + 3)
                    ctx.textAlign = "center"
                    ctx.fillText("BOW", cx, cy - radius - 7)
                    ctx.fillStyle = "#9fb1c0"
                    ctx.font = "11px monospace"
                    ctx.fillText("ROLL " + root.signed(roll, 1) + "°    PITCH " + root.signed(pitch, 1) + "°", cx, height - 4)
                }
            }
        }

        Rectangle {
            Layout.preferredWidth: 235
            Layout.fillHeight: true
            radius: 9
            color: "#101b27"
            border.color: "#2f4559"

            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                y: 10
                text: "LINEAR ACCELERATION · ±0.10 g"
                color: "#7f96aa"
                font.pixelSize: 10
                font.bold: true
            }

            Canvas {
                id: accelerationCanvas
                x: 14
                y: 32
                width: parent.width - 28
                height: 124
                onWidthChanged: requestPaint()
                onHeightChanged: requestPaint()

                onPaint: {
                    const ctx = getContext("2d")
                    ctx.reset()
                    ctx.clearRect(0, 0, width, height)
                    if (width <= 0 || height <= 0)
                        return
                    const cx = width / 2
                    const cy = height / 2
                    const radius = Math.min(width, height) * 0.42
                    const ax = Number(root.backendState.navigationMeasuredXG || 0)
                    const ay = Number(root.backendState.navigationMeasuredYG || 0)
                    const dotX = cx + root.clamp(ax / 0.10, -1, 1) * radius
                    const dotY = cy - root.clamp(ay / 0.10, -1, 1) * radius

                    ctx.strokeStyle = "#34506a"
                    ctx.lineWidth = 1
                    for (let ring = 1; ring <= 2; ++ring) {
                        ctx.beginPath()
                        ctx.arc(cx, cy, radius * ring / 2, 0, Math.PI * 2)
                        ctx.stroke()
                    }
                    ctx.beginPath()
                    ctx.moveTo(cx - radius, cy)
                    ctx.lineTo(cx + radius, cy)
                    ctx.moveTo(cx, cy - radius)
                    ctx.lineTo(cx, cy + radius)
                    ctx.stroke()

                    ctx.strokeStyle = "#ffaa46"
                    ctx.lineWidth = 3
                    ctx.beginPath()
                    ctx.moveTo(cx, cy)
                    ctx.lineTo(dotX, dotY)
                    ctx.stroke()
                    ctx.fillStyle = "#ffaa46"
                    ctx.beginPath()
                    ctx.arc(dotX, dotY, 7, 0, Math.PI * 2)
                    ctx.fill()

                    ctx.fillStyle = "#849aad"
                    ctx.font = "9px sans-serif"
                    ctx.textAlign = "center"
                    ctx.fillText("FWD", cx, 10)
                    ctx.fillText("REV", cx, height - 2)
                    ctx.textAlign = "left"
                    ctx.fillText("R", width - 12, cy + 3)
                    ctx.textAlign = "right"
                    ctx.fillText("L", 12, cy + 3)
                }
            }

            Column {
                x: 14
                y: 162
                width: parent.width - 28
                spacing: 3
                Text {
                    text: "X  " + root.signed(root.backendState.navigationMeasuredXG, 3) + " g"
                    color: "#e3ebf2"
                    font.pixelSize: 12
                    font.bold: true
                }
                Text {
                    text: "Y  " + root.signed(root.backendState.navigationMeasuredYG, 3) + " g"
                    color: "#e3ebf2"
                    font.pixelSize: 12
                    font.bold: true
                }
                Text {
                    text: "TOTAL  " + Number(root.backendState.navigationMeasuredMagnitudeG || 0).toFixed(3) + " g"
                    color: "#ffaa46"
                    font.pixelSize: 11
                }
                Text {
                    text: "YAW RATE  " + root.signed(root.backendState.navigationMeasuredYawDps, 1) + " °/s"
                    color: "#c58cff"
                    font.pixelSize: 10
                }
                Text {
                    text: Boolean(root.backendState.navigationImuReady)
                        ? "ZEROED · READY FOR ASSIST"
                        : "AUTO BASELINE · ZERO FOR ASSIST"
                    color: Boolean(root.backendState.navigationImuReady) ? "#57d4a5" : "#e2a463"
                    font.pixelSize: 9
                    font.bold: true
                }
                Text {
                    text: Boolean(root.backendState.navigationTiltCompensating)
                        ? "TILT COMPENSATION · " + Number(root.backendState.navigationTiltRateDps || 0).toFixed(1) + " °/s"
                        : "TRANSLATION MONITORING"
                    color: Boolean(root.backendState.navigationTiltCompensating) ? "#57d4a5" : "#7890a3"
                    font.pixelSize: 9
                    font.bold: true
                }
            }
        }
    }
}
