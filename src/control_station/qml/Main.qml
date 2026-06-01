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

    function handleKey(event, pressed) {
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
            color: "#e6ecf2"
            font.pixelSize: 28
            font.bold: true
        }

        Text {
            x: 20
            y: 50
            text: "Turn " + s.turn + "%  Thrust " + s.thrust + "%"
            color: "#9eb5c7"
            font.pixelSize: 14
        }

        Canvas {
            id: buoyCanvas
            anchors.fill: parent
            anchors.margins: 40
            onPaint: {
                const ctx = getContext("2d")
                ctx.reset()
                ctx.clearRect(0, 0, width, height)

                const cx = width / 2
                const cy = height / 2 + 10
                const scale = 120
                const turn = clamp(s.turn / 100.0, -1, 1)
                const thrust = clamp(s.thrust / 100.0, -1, 1)

                ctx.lineWidth = 1
                ctx.strokeStyle = "#3a4d61"
                ctx.beginPath()
                ctx.arc(cx, cy, 110, 0, Math.PI * 2)
                ctx.stroke()

                ctx.beginPath()
                ctx.moveTo(cx - 110, cy)
                ctx.lineTo(cx + 110, cy)
                ctx.moveTo(cx, cy - 110)
                ctx.lineTo(cx, cy + 110)
                ctx.stroke()

                const points = [
                    [cx, cy + 72],
                    [cx - 66, cy - 38],
                    [cx + 66, cy - 38]
                ]

                ctx.fillStyle = "#3d4c5f"
                ctx.strokeStyle = "#3a4d61"
                ctx.lineWidth = 2
                ctx.beginPath()
                ctx.moveTo(points[0][0], points[0][1])
                ctx.lineTo(points[1][0], points[1][1])
                ctx.lineTo(points[2][0], points[2][1])
                ctx.closePath()
                ctx.fill()
                ctx.stroke()

                function motor(x, y, value) {
                    const radius = 16 + Math.abs(value) * 0.12
                    ctx.beginPath()
                    ctx.fillStyle = value >= 0 ? "#3ddc97" : "#ffaa46"
                    ctx.arc(x, y, radius, 0, Math.PI * 2)
                    ctx.fill()
                    ctx.strokeStyle = "#e6ecf2"
                    ctx.lineWidth = 2
                    ctx.stroke()
                }

                motor(points[0][0], points[0][1], s.rearMotor || 0)
                motor(points[1][0], points[1][1], s.frontLeftMotor || 0)
                motor(points[2][0], points[2][1], s.frontRightMotor || 0)

                ctx.strokeStyle = "#50aaff"
                ctx.lineWidth = 6
                ctx.beginPath()
                ctx.moveTo(cx, cy)
                ctx.lineTo(cx + turn * scale, cy - thrust * scale)
                ctx.stroke()
                ctx.fillStyle = "#50aaff"
                ctx.beginPath()
                ctx.arc(cx + turn * scale, cy - thrust * scale, 8, 0, Math.PI * 2)
                ctx.fill()
            }
        }
    }

    Rectangle {
        id: statusPanel
        x: 36
        y: 448
        width: 470
        height: 516
        radius: 18
        color: "#182230"
        border.color: "#41556b"
        border.width: 1

        Text {
            x: 20
            y: 16
            text: "Buoy Status & Navigation"
            color: "#e6ecf2"
            font.pixelSize: 26
            font.bold: true
        }

        Column {
            x: 20
            y: 62
            width: parent.width - 40
            spacing: 8

            Repeater {
                model: [
                    { label: "Battery", value: s.batteryStatus },
                    { label: "Connection", value: s.serialStatus },
                    { label: "Mode", value: s.controllerMode || "idle" },
                    { label: "Location", value: s.currentLocation },
                    { label: "Target", value: s.targetLocation },
                    { label: "Hold", value: s.holdPosition ? "ON" : "OFF" },
                    { label: "Vector", value: s.turn + ", " + s.thrust },
                    { label: "Ack", value: s.ackVector || "N/A" },
                    { label: "Current", value: s.currentDraw }
                ]

                delegate: Rectangle {
                    width: parent.width
                    height: 38
                    color: "#0f1722"
                    radius: 8
                    border.color: "#395166"
                    border.width: 1

                    Text {
                        x: 12
                        y: 10
                        text: modelData.label
                        color: "#9eb5c7"
                        font.pixelSize: 13
                    }

                    Text {
                        x: parent.width - width - 12
                        y: 10
                        text: modelData.value
                        color: "#e6ecf2"
                        font.pixelSize: 13
                        elide: Text.ElideRight
                    }
                }
            }
        }
    }

    Rectangle {
        id: sciencePanel
        x: 532
        y: 28
        width: 1032
        height: 360
        radius: 18
        color: "#182230"
        border.color: "#41556b"
        border.width: 1

        Text {
            x: 20
            y: 16
            text: "Science & Audio"
            color: "#e6ecf2"
            font.pixelSize: 26
            font.bold: true
        }

        Row {
            x: 20
            y: 62
            width: parent.width - 40
            height: 220
            spacing: 18

            Column {
                width: 360
                spacing: 6
                Repeater {
                    model: [
                        { label: "Depth", value: s.currentDepth },
                        { label: "Water Temp", value: s.waterTemperature },
                        { label: "Air Temp", value: s.airTemperature },
                        { label: "IMU Accel", value: s.imuAccel },
                        { label: "IMU Gyro", value: s.imuGyro },
                        { label: "IMU Temp", value: s.imuTemperature },
                        { label: "IMU UDP Loss", value: s.imuUdpLoss }
                    ]

                    delegate: Rectangle {
                        width: 360
                        height: 24
                        color: "transparent"
                        Text {
                            x: 0
                            text: modelData.label
                            color: "#9eb5c7"
                            font.pixelSize: 13
                        }
                        Text {
                            anchors.right: parent.right
                            text: modelData.value
                            color: "#e6ecf2"
                            font.pixelSize: 13
                            elide: Text.ElideRight
                        }
                    }
                }
            }

            Rectangle {
                width: parent.width - 398
                height: 220
                radius: 12
                color: "#0f1722"
                border.color: "#395166"
                border.width: 1

                MouseArea {
                    id: muteMouse
                    anchors.fill: parent
                    onClicked: backend.toggleAudioMute()
                    cursorShape: Qt.PointingHandCursor
                }

                Text {
                    x: 14
                    y: 10
                    text: "Stream: " + s.audioStream
                    color: "#e6ecf2"
                    font.pixelSize: 13
                    elide: Text.ElideRight
                    width: parent.width - 28
                }

                Text {
                    x: 14
                    y: 34
                    text: "Level: " + s.audioLevel
                    color: "#e6ecf2"
                    font.pixelSize: 13
                }

                Rectangle {
                    x: 14
                    y: 62
                    width: 54
                    height: 42
                    radius: 8
                    color: backend.audioMuted ? "#ffaa46" : "#3ddc97"
                    border.color: "#e6ecf2"
                    border.width: 1

                    Canvas {
                        anchors.fill: parent
                        onPaint: {
                            const ctx = getContext("2d")
                            ctx.reset()
                            ctx.clearRect(0, 0, width, height)
                            ctx.strokeStyle = "#0f1722"
                            ctx.fillStyle = "#0f1722"
                            ctx.lineWidth = 2
                            ctx.beginPath()
                            ctx.moveTo(10, 16)
                            ctx.lineTo(18, 16)
                            ctx.lineTo(25, 10)
                            ctx.lineTo(25, 30)
                            ctx.lineTo(18, 24)
                            ctx.lineTo(10, 24)
                            ctx.closePath()
                            ctx.fill()
                            ctx.stroke()
                            if (backend.audioMuted) {
                                ctx.strokeStyle = "#0f1722"
                                ctx.lineWidth = 3
                                ctx.beginPath()
                                ctx.moveTo(30, 10)
                                ctx.lineTo(46, 32)
                                ctx.moveTo(30, 32)
                                ctx.lineTo(46, 10)
                                ctx.stroke()
                            } else {
                                ctx.strokeStyle = "#0f1722"
                                ctx.lineWidth = 2
                                ctx.beginPath()
                                ctx.arc(36, 21, 8, -0.7, 0.7)
                                ctx.stroke()
                                ctx.beginPath()
                                ctx.arc(42, 21, 13, -0.8, 0.8)
                                ctx.stroke()
                            }
                        }
                    }
                }

                Canvas {
                    id: audioCanvas
                    x: 84
                    y: 62
                    width: parent.width - 98
                    height: 140
                    onPaint: {
                        const ctx = getContext("2d")
                        ctx.reset()
                        ctx.clearRect(0, 0, width, height)
                        ctx.fillStyle = "#182230"
                        ctx.fillRect(0, 0, width, height)
                        ctx.strokeStyle = "#395166"
                        ctx.lineWidth = 1
                        ctx.strokeRect(0, 0, width, height)
                        ctx.strokeStyle = "#395166"
                        ctx.beginPath()
                        ctx.moveTo(0, height / 2)
                        ctx.lineTo(width, height / 2)
                        ctx.stroke()

                        const waveform = backend.audioWaveform
                        if (!waveform || waveform.length < 2) {
                            ctx.fillStyle = "#ffaa46"
                            ctx.fillText("No audio yet", 12, 20)
                            return
                        }

                        const step = Math.max(1, Math.floor(waveform.length / Math.max(2, width - 16)))
                        const scaleY = height * 0.38
                        ctx.strokeStyle = "#50aaff"
                        ctx.lineWidth = 2
                        ctx.beginPath()
                        let drawIndex = 0
                        for (let i = 0; i < waveform.length; i += step) {
                            const x = 8 + drawIndex
                            if (x >= width - 8) {
                                break
                            }
                            const value = Math.max(-1.0, Math.min(1.0, waveform[i]))
                            const y = (height / 2) - value * scaleY
                            if (drawIndex === 0) {
                                ctx.moveTo(x, y)
                            } else {
                                ctx.lineTo(x, y)
                            }
                            drawIndex++
                        }
                        ctx.stroke()
                    }
                }
            }
        }
    }

    Rectangle {
        id: dashboardPanel
        x: 532
        y: 408
        width: 1032
        height: 556
        radius: 18
        color: "#182230"
        border.color: "#41556b"
        border.width: 1

        Text {
            x: 20
            y: 16
            text: "Communications / Science Analysis"
            color: "#e6ecf2"
            font.pixelSize: 26
            font.bold: true
        }

        TabBar {
            id: dashboardTabs
            x: 16
            y: 54
            width: 280
            currentIndex: backend.dashboardTab === "analysis" ? 1 : 0
            onCurrentIndexChanged: backend.setDashboardTab(currentIndex === 0 ? "logs" : "analysis")

            TabButton { text: "Logs" }
            TabButton { text: "Science Analysis" }
        }

        StackLayout {
            x: 16
            y: 96
            width: parent.width - 32
            height: parent.height - 112
            currentIndex: backend.dashboardTab === "analysis" ? 1 : 0

            Item {
                ListView {
                    id: logList
                    anchors.fill: parent
                    clip: true
                    model: backend.commLog
                    spacing: 6
                    boundsBehavior: Flickable.StopAtBounds
                    onCountChanged: positionViewAtEnd()

                    delegate: Rectangle {
                        width: logList.width
                        color: "transparent"
                        implicitHeight: logText.implicitHeight + 10

                        Text {
                            id: logText
                            x: 8
                            y: 4
                            width: parent.width - 16
                            text: modelData
                            color: "#e6ecf2"
                            font.pixelSize: 13
                            wrapMode: Text.Wrap
                        }
                    }
                }
            }

            Item {
                Canvas {
                    id: mapCanvas
                    anchors.fill: parent
                    onPaint: {
                        const ctx = getContext("2d")
                        ctx.reset()
                        ctx.clearRect(0, 0, width, height)
                        ctx.fillStyle = "#0f1722"
                        ctx.fillRect(0, 0, width, height)
                        ctx.strokeStyle = "#395166"
                        ctx.lineWidth = 1
                        ctx.strokeRect(0, 0, width, height)

                        const samples = backend.scienceHistory
                        if (!samples || samples.length === 0) {
                            ctx.fillStyle = "#ffaa46"
                            ctx.fillText("Waiting for science samples...", 16, 24)
                            return
                        }

                        let latMin = samples[0].latitude
                        let latMax = samples[0].latitude
                        let lonMin = samples[0].longitude
                        let lonMax = samples[0].longitude
                        for (let i = 1; i < samples.length; ++i) {
                            const sample = samples[i]
                            latMin = Math.min(latMin, sample.latitude)
                            latMax = Math.max(latMax, sample.latitude)
                            lonMin = Math.min(lonMin, sample.longitude)
                            lonMax = Math.max(lonMax, sample.longitude)
                        }

                        const latSpan = Math.max(latMax - latMin, 0.00001)
                        const lonSpan = Math.max(lonMax - lonMin, 0.00001)
                        const centerLat = (latMin + latMax) / 2.0
                        const centerLon = (lonMin + lonMax) / 2.0
                        const scaleX = (width - 80) / lonSpan
                        const scaleY = (height - 80) / latSpan
                        const scale = Math.min(scaleX, scaleY, 180000.0)

                        ctx.strokeStyle = "#395166"
                        ctx.beginPath()
                        ctx.moveTo(width / 2, 20)
                        ctx.lineTo(width / 2, height - 20)
                        ctx.moveTo(20, height / 2)
                        ctx.lineTo(width - 20, height / 2)
                        ctx.stroke()

                        ctx.strokeStyle = "#50aaff"
                        ctx.lineWidth = 2
                        ctx.beginPath()
                        for (let i = 0; i < samples.length; ++i) {
                            const sample = samples[i]
                            const x = width / 2 + (sample.longitude - centerLon) * scale
                            const y = height / 2 - (sample.latitude - centerLat) * scale
                            if (i === 0) {
                                ctx.moveTo(x, y)
                            } else {
                                ctx.lineTo(x, y)
                            }
                        }
                        ctx.stroke()

                        for (let i = 0; i < samples.length; ++i) {
                            const sample = samples[i]
                            const x = width / 2 + (sample.longitude - centerLon) * scale
                            const y = height / 2 - (sample.latitude - centerLat) * scale
                            const depthRadius = 4 + Math.min(10, sample.depth_m * 1.5)
                            const audio = Math.max(0, Math.min(100, sample.audio_level_percent)) / 100.0
                            let r, g, b
                            if (audio < 0.5) {
                                const mix = audio / 0.5
                                r = 61 + (255 - 61) * mix
                                g = 220 + (170 - 220) * mix
                                b = 151 + (70 - 151) * mix
                            } else {
                                const mix = (audio - 0.5) / 0.5
                                r = 255
                                g = 170 + (105 - 170) * mix
                                b = 70 + (97 - 70) * mix
                            }
                            ctx.fillStyle = "rgb(" + Math.round(r) + "," + Math.round(g) + "," + Math.round(b) + ")"
                            ctx.beginPath()
                            ctx.arc(x, y, depthRadius, 0, Math.PI * 2)
                            ctx.fill()
                            ctx.strokeStyle = "#e6ecf2"
                            ctx.stroke()
                        }

                        const latest = samples[samples.length - 1]
                        ctx.fillStyle = "#e6ecf2"
                        ctx.fillText("Latest depth: " + latest.depth_m.toFixed(2) + " m", 16, 24)
                        ctx.fillText("Latest audio: " + latest.audio_level_percent.toFixed(1) + "%", 240, 24)
                        ctx.fillText("Location: " + latest.latitude.toFixed(6) + ", " + latest.longitude.toFixed(6), 460, 24)
                    }
                }
            }
        }
    }

    Connections {
        target: backend
        function onStateChanged() {
            buoyCanvas.requestPaint()
            audioCanvas.requestPaint()
            mapCanvas.requestPaint()
        }
    }
}
