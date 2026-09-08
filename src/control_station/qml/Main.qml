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

    function findInterfaceIndex(value) {
        const interfaces = backend.networkInterfaces || []
        for (let i = 0; i < interfaces.length; ++i) {
            if (interfaces[i].value === value) {
                return i
            }
        }
        return 0
    }

    function dashboardTabIndex(tab) {
        if (tab === "logs") return 1
        if (tab === "map") return 2
        if (tab === "analysis") return 3
        if (tab === "motors") return 4
        return 0
    }

    function dashboardTabForIndex(index) {
        if (index === 1) return "logs"
        if (index === 2) return "map"
        if (index === 3) return "analysis"
        if (index === 4) return "motors"
        return "science"
    }

    function isSerialConnected() {
        return String(s.serialStatus || "").indexOf("Connected") === 0
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

    function parseTemperature(value) {
        if (value === null || value === undefined || value === "N/A" || value === "Unknown") {
            return NaN
        }
        const parsed = Number(value)
        return Number.isFinite(parsed) ? parsed : NaN
    }

    function temperatureColor(tempC, depthM) {
        const numericTemp = parseTemperature(tempC)
        if (Number.isFinite(numericTemp)) {
            const cold = 8.0
            const hot = 32.0
            const mix = clamp((numericTemp - cold) / (hot - cold), 0, 1)
            const r = Math.round(54 + (255 - 54) * mix)
            const g = Math.round(160 + (110 - 160) * mix)
            const b = Math.round(228 + (66 - 228) * mix)
            return "rgb(" + r + "," + g + "," + b + ")"
        }

        const depthMix = clamp((depthM || 0.0) / 8.0, 0, 1)
        const r2 = Math.round(61 + (122 - 61) * depthMix)
        const g2 = Math.round(180 + (120 - 180) * depthMix)
        const b2 = Math.round(220 + (150 - 220) * depthMix)
        return "rgb(" + r2 + "," + g2 + "," + b2 + ")"
    }

    function latestScienceSample() {
        const samples = backend.scienceHistory || []
        if (samples.length === 0) {
            return null
        }
        return samples[samples.length - 1]
    }

    function mapCenter() {
        const latest = latestScienceSample()
        if (latest && Number.isFinite(latest.latitude) && Number.isFinite(latest.longitude)) {
            return { latitude: latest.latitude, longitude: latest.longitude }
        }
        return { latitude: 37.7749, longitude: -122.4194 }
    }

    function scienceBounds(samples) {
        if (!samples || samples.length === 0) {
            return null
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

        let latSpan = Math.max(latMax - latMin, 0.00008)
        let lonSpan = Math.max(lonMax - lonMin, 0.00008)
        const paddingScale = 0.20
        latSpan *= 1.0 + paddingScale
        lonSpan *= 1.0 + paddingScale

        const centerLat = (latMin + latMax) / 2.0
        const centerLon = (lonMin + lonMax) / 2.0
        return {
            minLat: centerLat - latSpan / 2.0,
            maxLat: centerLat + latSpan / 2.0,
            minLon: centerLon - lonSpan / 2.0,
            maxLon: centerLon + lonSpan / 2.0,
            latSpan: latSpan,
            lonSpan: lonSpan,
            centerLat: centerLat,
            centerLon: centerLon
        }
    }

    function meshColor(temperature, depth) {
        return temperatureColor(temperature, depth)
    }

    function meshPointColor(point) {
        return meshColor(point.temperature, point.depth)
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
                x: 36
                y: 448
                width: 470
                height: 78
                color: "transparent"

                Rectangle {
                    anchors.fill: parent
                    radius: 8
                    color: "#0f1722"
                    border.color: "#395166"
                    border.width: 1

                    Column {
                        anchors.fill: parent
                        anchors.margins: 10
                        spacing: 8

                        Text {
                            text: "Communications"
                            color: "#e6ecf2"
                            font.pixelSize: 13
                            font.bold: true
                        }

                        Row {
                            width: parent.width
                            spacing: 8

                            TextField {
                                id: commTextField
                                width: parent.width - 88
                                height: 30
                                placeholderText: "Type text to send to the buoy"
                                color: "#e6ecf2"
                                selectionColor: "#3ddc97"
                                selectedTextColor: "#0f1722"
                                placeholderTextColor: "#9eb5c7"
                                background: Rectangle {
                                    radius: 6
                                    color: "#182230"
                                    border.color: "#395166"
                                    border.width: 1
                                }
                                onAccepted: sendCommText()
                            }

                            Button {
                                text: "Send"
                                width: 80
                                height: 30
                                enabled: isSerialConnected() && commTextField.text.trim().length > 0
                                onClicked: sendCommText()
                            }
                        }
                    }
                }

                function sendCommText() {
                    const message = commTextField.text.trim()
                    if (message.length === 0) {
                        return
                    }
                    backend.sendText(message)
                }
            }
    Rectangle {
        id: statusPanel
        x: 36
        y: 532
        width: 470
        height: 468
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
            spacing: 6

            Rectangle {
                width: parent.width
                height: 44
                color: "transparent"

                Text {
                    x: 0
                    y: 12
                    text: "Source interface"
                    color: "#9eb5c7"
                    font.pixelSize: 13
                }

                ComboBox {
                    x: 120
                    width: parent.width - 220
                    height: 32
                    model: backend.networkInterfaces
                    textRole: "label"
                    valueRole: "value"
                    currentIndex: findInterfaceIndex(backend.wifiLocalIp)
                    onActivated: backend.setWifiLocalIp(currentValue)
                }

                Rectangle {
                    x: parent.width - 92
                    y: 6
                    width: 92
                    height: 30
                    radius: 8
                    color: backend.connectionButtonLabel === "Disconnect" ? "#3ddc97" : "#ffaa46"
                    border.color: "#e6ecf2"
                    border.width: 1
                    opacity: backend.connectionButtonLabel === "Connecting..." || backend.connectionButtonLabel === "Reconnecting..." ? 0.8 : 1.0

                    MouseArea {
                        anchors.fill: parent
                        onClicked: backend.toggleConnection()
                        cursorShape: Qt.PointingHandCursor
                        enabled: backend.connectionButtonLabel !== "Connecting..." && backend.connectionButtonLabel !== "Reconnecting..."
                    }

                    Text {
                        anchors.centerIn: parent
                        text: backend.connectionButtonLabel
                        color: "#0f1722"
                        font.pixelSize: 12
                        font.bold: true
                    }            }
        }
        Repeater {
                model: [
                    { label: "Connection", value: s.serialStatus },
                    { label: "Battery", value: s.batteryStatus },
                    { label: "Mode", value: s.controllerMode || "idle" },
                    { label: "Location", value: s.currentLocation },
                    { label: "Target", value: s.targetLocation },
                    { label: "Hold", value: s.holdPosition ? "ON" : "OFF" },
                    { label: "Vector", value: s.turn + ", " + s.thrust },
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

    Rectangle {
        id: legacySciencePanel
        visible: false
        x: 532
        y: 28
        width: 1032
        height: 360
        z: 1
        clip: true
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
                        }            }
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
                             }            }
        }
                }

                Canvas {
                    id: legacyAudioCanvas
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
                    }            }
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
            }

            TabButton { text: "Science & Audio" }
            TabButton { text: "Logs" }
            TabButton { text: "Map" }
            TabButton { text: "Bottom Mesh" }
            TabButton { text: "Motor Test" }
        }

        StackLayout {
            x: 16
            y: 96
            width: parent.width - 32
            height: parent.height - 112
            currentIndex: dashboardTabIndex(backend.dashboardTab)

            ScienceAudioPanel {
                Layout.fillWidth: true
                Layout.fillHeight: true
                backendObject: backend
                backendState: s
            }

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
                        }            }
        }
        }

            Item {
                Rectangle {
                    anchors.fill: parent
                    color: "#0b1119"
                    radius: 12
                    border.color: "#395166"
                    border.width: 1

                    Canvas {
                        id: mapCanvas
                        anchors.fill: parent
                        anchors.margins: 1
                        onPaint: {
                            const ctx = getContext("2d")
                            ctx.reset()
                            ctx.clearRect(0, 0, width, height)

                            const samples = backend.scienceHistory || []
                            const center = mapCenter()
                            const pixelsPerDegree = 19000
                            const baseScale = samples.length > 1 ? clamp(16 + samples.length * 1.5, 16, 32) : 22

                            ctx.fillStyle = "#102030"
                            ctx.fillRect(0, 0, width, height)

                            const gradient = ctx.createLinearGradient(0, 0, width, height)
                            gradient.addColorStop(0.0, "#0d1722")
                            gradient.addColorStop(0.5, "#122438")
                            gradient.addColorStop(1.0, "#0a1119")
                            ctx.fillStyle = gradient
                            ctx.fillRect(0, 0, width, height)

                            ctx.strokeStyle = "#213246"
                            ctx.lineWidth = 1
                            for (let x = -width; x < width * 2; x += 80) {
                                ctx.beginPath()
                                ctx.moveTo(x, 0)
                                ctx.lineTo(x + width * 0.15, height)
                                ctx.stroke()
                            }
                            for (let y = -height; y < height * 2; y += 80) {
                                ctx.beginPath()
                                ctx.moveTo(0, y)
                                ctx.lineTo(width, y + height * 0.05)
                                ctx.stroke()
                            }

                            ctx.fillStyle = "#9eb5c7"
                            ctx.font = "13px sans-serif"
                            ctx.fillText("Map view", 16, 24)
                            ctx.fillStyle = "#6f8599"
                            ctx.fillText("GPS track and sample markers centered on the buoy path", 16, 42)

                            function project(lat, lon) {
                                return {
                                    x: width / 2 + (lon - center.longitude) * pixelsPerDegree * baseScale / 22.0,
                                    y: height / 2 - (lat - center.latitude) * pixelsPerDegree * baseScale / 22.0
                                }
                            }

                            if (samples.length > 0) {
                                const trail = []
                                for (let i = 0; i < samples.length; ++i) {
                                    const sample = samples[i]
                                    if (!Number.isFinite(sample.latitude) || !Number.isFinite(sample.longitude)) {
                                        continue
                                    }
                                    trail.push(project(sample.latitude, sample.longitude))
                                }

                                if (trail.length > 1) {
                                    ctx.strokeStyle = "#50aaff"
                                    ctx.lineWidth = 4
                                    ctx.lineJoin = "round"
                                    ctx.lineCap = "round"
                                    ctx.beginPath()
                                    for (let i = 0; i < trail.length; ++i) {
                                        const pt = trail[i]
                                        if (i === 0) {
                                            ctx.moveTo(pt.x, pt.y)
                                        } else {
                                            ctx.lineTo(pt.x, pt.y)
                                        }
                                    }
                                    ctx.stroke()
                                }

                                for (let i = 0; i < samples.length; ++i) {
                                    const sample = samples[i]
                                    if (!Number.isFinite(sample.latitude) || !Number.isFinite(sample.longitude)) {
                                        continue
                                    }
                                    const point = project(sample.latitude, sample.longitude)
                                    ctx.fillStyle = temperatureColor(sample.water_temperature_c, sample.depth_m)
                                    ctx.beginPath()
                                    ctx.arc(point.x, point.y, 7, 0, Math.PI * 2)
                                    ctx.fill()
                                    ctx.strokeStyle = "#e6ecf2"
                                    ctx.lineWidth = 1
                                    ctx.stroke()
                                }

                                const latest = samples[samples.length - 1]
                                const latestPoint = project(latest.latitude, latest.longitude)
                                ctx.strokeStyle = "#e6ecf2"
                                ctx.lineWidth = 2
                                ctx.beginPath()
                                ctx.arc(latestPoint.x, latestPoint.y, 12, 0, Math.PI * 2)
                                ctx.stroke()
                                ctx.fillStyle = "#e6ecf2"
                                ctx.beginPath()
                                ctx.arc(latestPoint.x, latestPoint.y, 3, 0, Math.PI * 2)
                                ctx.fill()
                            } else {
                                ctx.fillStyle = "#ffaa46"
                                ctx.fillText("Waiting for GPS samples to place the track on the map.", 16, 66)
                            }            }
        }
    }
                }

            Item {
                Canvas {
                    id: analysisCanvas
                    anchors.fill: parent
                    onPaint: {
                        const ctx = getContext("2d")
                        ctx.reset()
                        ctx.clearRect(0, 0, width, height)
                        ctx.fillStyle = "#09111a"
                        ctx.fillRect(0, 0, width, height)
                        ctx.strokeStyle = "#395166"
                        ctx.lineWidth = 1
                        ctx.strokeRect(0, 0, width, height)
                        ctx.font = "13px sans-serif"

                        const samples = backend.scienceHistory || []
                        if (samples.length === 0) {
                            ctx.fillStyle = "#ffaa46"
                            ctx.fillText("Waiting for depth samples to build the mesh...", 16, 24)
                            return
                        }

                        const bounds = scienceBounds(samples)
                        const depthValues = []
                        let depthMin = samples[0].depth_m
                        let depthMax = samples[0].depth_m
                        let temperatureMin = Number.isFinite(samples[0].water_temperature_c) ? samples[0].water_temperature_c : NaN
                        let temperatureMax = Number.isFinite(samples[0].water_temperature_c) ? samples[0].water_temperature_c : NaN
                        for (let i = 0; i < samples.length; ++i) {
                            const sample = samples[i]
                            depthMin = Math.min(depthMin, sample.depth_m)
                            depthMax = Math.max(depthMax, sample.depth_m)
                            depthValues.push(sample.depth_m)
                            if (Number.isFinite(sample.water_temperature_c)) {
                                temperatureMin = Number.isFinite(temperatureMin) ? Math.min(temperatureMin, sample.water_temperature_c) : sample.water_temperature_c
                                temperatureMax = Number.isFinite(temperatureMax) ? Math.max(temperatureMax, sample.water_temperature_c) : sample.water_temperature_c
                            }
                        }

                        const latSpan = bounds.latSpan
                        const lonSpan = bounds.lonSpan
                        const gridCols = Math.max(5, Math.min(18, Math.ceil(Math.sqrt(samples.length) + 2)))
                        const gridRows = Math.max(5, Math.min(18, Math.ceil(samples.length / gridCols) + 2))
                        const nodes = []

                        function sampleField(latitude, longitude, fieldName) {
                            let weighted = 0.0
                            let weightTotal = 0.0
                            const minInfluence = Math.max(latSpan, lonSpan)
                            for (let i = 0; i < samples.length; ++i) {
                                const sample = samples[i]
                                const latDelta = sample.latitude - latitude
                                const lonDelta = sample.longitude - longitude
                                const distance = Math.max(0.00001, Math.sqrt(latDelta * latDelta + lonDelta * lonDelta) / minInfluence)
                                const weight = 1.0 / (distance * distance)
                                const rawValue = fieldName === "temperature" ? sample.water_temperature_c : sample.depth_m
                                if (!Number.isFinite(rawValue)) {
                                    continue
                                }
                                weighted += rawValue * weight
                                weightTotal += weight
                            }
                            if (weightTotal === 0.0) {
                                return fieldName === "temperature" ? NaN : depthMin
                            }
                            return weighted / weightTotal
                        }

                        function projectPoint(nx, ny, depth) {
                            const tileX = Math.max(14, Math.min(26, width / 40))
                            const tileY = tileX * 0.55
                            const depthScale = Math.max(8, Math.min(26, height / 20))
                            const isoX = (nx - ny) * tileX
                            const isoY = (nx + ny) * tileY
                            return {
                                x: width * 0.50 + isoX,
                                y: height * 0.26 + isoY + depth * depthScale
                            }
                        }

                        for (let row = 0; row < gridRows; ++row) {
                            const rowPoints = []
                            for (let col = 0; col < gridCols; ++col) {
                                const nx = gridCols === 1 ? 0.5 : col / (gridCols - 1)
                                const ny = gridRows === 1 ? 0.5 : row / (gridRows - 1)
                                const latitude = bounds.maxLat - ny * latSpan
                                const longitude = bounds.minLon + nx * lonSpan
                                const depth = sampleField(latitude, longitude, "depth")
                                const temperature = sampleField(latitude, longitude, "temperature")
                                const point = projectPoint(nx - 0.5, ny - 0.5, depth)
                                rowPoints.push({
                                    x: point.x,
                                    y: point.y,
                                    depth: depth,
                                    temperature: temperature,
                                    latitude: latitude,
                                    longitude: longitude
                                })
                            }
                            nodes.push(rowPoints)
                        }

                        ctx.fillStyle = "#e6ecf2"
                        ctx.fillText("Lake bottom topology mesh", 16, 24)
                        const latest = samples[samples.length - 1]
                        ctx.fillStyle = "#9eb5c7"
                        ctx.fillText(
                            "Bottom depth range: " + depthMin.toFixed(2) + " m to " + depthMax.toFixed(2) + " m",
                            16,
                            42
                        )
                        ctx.fillText(
                            "Latest temp: " + (Number.isFinite(latest.water_temperature_c) ? latest.water_temperature_c.toFixed(1) + " C" : "N/A"),
                            320,
                            42
                        )

                        for (let row = gridRows - 2; row >= 0; --row) {
                            for (let col = 0; col < gridCols - 1; ++col) {
                                const a = nodes[row][col]
                                const b = nodes[row][col + 1]
                                const c = nodes[row + 1][col + 1]
                                const d = nodes[row + 1][col]
                                const depth = (a.depth + b.depth + c.depth + d.depth) / 4.0
                                const temperatureSamples = [
                                    a.temperature,
                                    b.temperature,
                                    c.temperature,
                                    d.temperature,
                                ]
                                let temperatureTotal = 0.0
                                let tempCount = 0
                                for (let i = 0; i < temperatureSamples.length; ++i) {
                                    const value = temperatureSamples[i]
                                    if (!Number.isFinite(value)) {
                                        continue
                                    }
                                    temperatureTotal += value
                                    tempCount += 1
                                }
                                const averageTemp = tempCount > 0 ? temperatureTotal / tempCount : NaN
                                ctx.fillStyle = temperatureColor(averageTemp, depth)
                                ctx.globalAlpha = 0.72
                                ctx.beginPath()
                                ctx.moveTo(a.x, a.y)
                                ctx.lineTo(b.x, b.y)
                                ctx.lineTo(c.x, c.y)
                                ctx.lineTo(d.x, d.y)
                                ctx.closePath()
                                ctx.fill()
                                ctx.globalAlpha = 1.0
                            }
                        }

                        ctx.strokeStyle = "#2d4156"
                        ctx.lineWidth = 1
                        for (let row = 0; row < gridRows; ++row) {
                            ctx.beginPath()
                            for (let col = 0; col < gridCols; ++col) {
                                const point = nodes[row][col]
                                if (col === 0) {
                                    ctx.moveTo(point.x, point.y)
                                } else {
                                    ctx.lineTo(point.x, point.y)
                                }
                            }
                            ctx.stroke()
                        }
                        for (let col = 0; col < gridCols; ++col) {
                            ctx.beginPath()
                            for (let row = 0; row < gridRows; ++row) {
                                const point = nodes[row][col]
                                if (row === 0) {
                                    ctx.moveTo(point.x, point.y)
                                } else {
                                    ctx.lineTo(point.x, point.y)
                                }
                            }
                            ctx.stroke()
                        }

                        for (let row = 0; row < gridRows; ++row) {
                            for (let col = 0; col < gridCols; ++col) {
                                const point = nodes[row][col]
                                const radius = 2.5 + Math.min(4.5, Math.max(0.0, point.depth * 0.45))
                                ctx.fillStyle = temperatureColor(point.temperature, point.depth)
                                ctx.beginPath()
                                ctx.arc(point.x, point.y, radius, 0, Math.PI * 2)
                                ctx.fill()
                                ctx.strokeStyle = "#ecf2f7"
                                ctx.lineWidth = 1
                                ctx.stroke()
                            }
                        }

                        ctx.fillStyle = "#50aaff"
                        ctx.beginPath()
                        ctx.arc(nodes[gridRows - 1][gridCols - 1].x, nodes[gridRows - 1][gridCols - 1].y, 6, 0, Math.PI * 2)
                        ctx.fill()
                    }            }
        }
            MotorTestPanel {
                id: motorTestPanel
                Layout.fillWidth: true
                Layout.fillHeight: true
                backendObject: backend
                backendState: s
            }
        }
    }
    Connections {
        target: backend
        function onStateChanged() {
            buoyCanvas.requestPaint()
            mapCanvas.requestPaint()
            analysisCanvas.requestPaint()
        }
    }
}


