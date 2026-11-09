import QtQuick

Rectangle {
    id: root
    property string title: ""
    property string tab: ""
    property var backendObject
    property var backendState: ({})
    readonly property var s: backendState
    readonly property real currentAmps: numericPrefix(s.currentDraw)
    readonly property real batteryLevel: percentPrefix(s.batteryStatus)
    readonly property real batteryVoltage: numericPrefix(s.batteryStatus)
    readonly property var powerSamples: s.powerHistory || []
    readonly property real cardTotal: Number((s.sdCardCapacity || {}).totalBytes) || 0
    readonly property real cardUsed: Number((s.sdCardCapacity || {}).usedBytes) || 0
    readonly property real cardFree: Number((s.sdCardCapacity || {}).freeBytes) || Math.max(0, cardTotal - cardUsed)
    readonly property real cardFraction: cardTotal > 0 ? Math.max(0, Math.min(1, cardUsed / cardTotal)) : 0
    readonly property real heading: Number(s.magnetometerTiltHeadingDeg !== null && s.magnetometerTiltHeadingDeg !== undefined
                                   ? s.magnetometerTiltHeadingDeg : s.magnetometerHeadingDeg)
    readonly property bool headingAvailable: s.telemetrySource === "Buoy Wi-Fi"
                                             && (s.magnetometerTiltHeadingDeg !== null && s.magnetometerTiltHeadingDeg !== undefined
                                                 || s.magnetometerHeadingDeg !== null && s.magnetometerHeadingDeg !== undefined)
    readonly property bool courseAvailable: Boolean(s.navigationGpsCourseAvailable && s.navigationGpsSpeedAvailable)
                                            && Number(s.navigationGpsSpeedMps) >= 0.3
    readonly property var motorEnergy: s.motorEnergyWh || ({})
    readonly property real motorEnergyMax: Math.max(Number(motorEnergy.rear) || 0, Number(motorEnergy.left) || 0, Number(motorEnergy.right) || 0, 0.001)
    readonly property string chartKey: JSON.stringify([title, powerSamples, batteryVoltage,
                                                       heading, headingAvailable, s.navigationGpsCourseDeg, courseAvailable,
                                                       s.navigationGpsSpeedMps, s.currentLocation])
    onChartKeyChanged: if (chart) chart.requestPaint()

    function numericPrefix(value) {
        const match = String(value === undefined || value === null ? "" : value).match(/-?\d+(?:\.\d+)?/)
        return match ? Number(match[0]) : NaN
    }
    function percentPrefix(value) {
        const match = String(value === undefined || value === null ? "" : value).match(/(\d+(?:\.\d+)?)\s*%/)
        return match ? Number(match[1]) : NaN
    }
    function sizeLabel(bytes) {
        if (!Number.isFinite(bytes) || bytes < 0) return "--"
        if (bytes >= 1073741824) return (bytes / 1073741824).toFixed(1) + " GB"
        return (bytes / 1048576).toFixed(0) + " MB"
    }
    readonly property bool motorEstimateAvailable: String(s.motorEnergyEstimatorStatus || "").startsWith("Estimated")
    readonly property bool powerTrendAvailable: powerSamples.length > 1
                                               && Number(powerSamples[0].t) !== Number(powerSamples[powerSamples.length - 1].t)
    readonly property var radioChannels: s.radioChannels || []
    readonly property var buoyState: s.radioBuoyState || null

    function channelValue(channel) {
        const value = Number(radioChannels[channel - 1])
        return Number.isFinite(value) && value >= 800 && value <= 2200 ? value : null
    }
    function routeSummary() {
        const value = channelValue(6)
        const mode = String(s.telemetryMode || "Unknown").toUpperCase()
        if (value !== null) return "Route " + mode + " · SF / CH6 " + (value >= 1700 ? "HIGH" : "LOW/MID")
        return "Route " + mode + " · SF / CH6 waiting"
    }
    function switchSummary(channel, name) {
        const value = channelValue(channel)
        if (value === null) return name + " / CH" + channel + " · waiting"
        const position = value < 1300 ? "LOW" : value >= 1700 ? "HIGH" : "MID"
        if (channel === 5) return name + " / CH5 " + position + " · " + (value >= 1700 ? "arm request" : "disarm")
        if (channel === 7) return name + " / CH7 " + position + " · " + (value >= 1700 ? "mission request (unavailable)" : "manual")
        return name + " / CH" + channel + " " + position
    }
    function buoyArmSummary() {
        if (buoyState) return buoyState.armed ? "ARMED" : "DISARMED"
        return String(s.radioArmStatus || "Arm state waiting").toUpperCase()
    }
    function buoyModeSummary() {
        if (buoyState && buoyState.mode) return String(buoyState.mode)
        return String(s.radioBuoyMode || s.buoyMode || "Buoy mode waiting")
    }
    function controlSummary() {
        const source = String((buoyState && buoyState.control_source) || s.controlSource || "Unknown")
        return "Control: " + buoyModeSummary().toUpperCase() + " · " + source
    }
    function linkQualityPercent() {
        const match = String(s.radioLinkQuality || "").match(/(\d+(?:\.\d+)?)\s*%/)
        return match ? Number(match[1]) : -1
    }
    function linkQualitySummary() {
        const status = String(s.radioLinkStatus || "").toLowerCase()
        const quality = linkQualityPercent()
        if (status.includes("stale")) return "STALE"
        return quality >= 0 ? Math.round(quality) + "% LQ" : "LINK --"
    }
    function linkQualityColor() {
        const quality = linkQualityPercent()
        if (String(s.radioLinkStatus || "").toLowerCase().includes("stale") || quality < 0) return "#91a8b9"
        return quality >= 70 ? "#58d7a5" : quality >= 40 ? "#f1bd68" : "#f07878"
    }
    function linkQualityBackground() {
        const quality = linkQualityPercent()
        if (String(s.radioLinkStatus || "").toLowerCase().includes("stale") || quality < 0) return "#233342"
        return quality >= 70 ? "#17382f" : quality >= 40 ? "#3b3021" : "#3a2024"
    }
    function telemetrySummary() {
        const packets = Number(s.radioTelemetryPackets) || 0
        const status = String(s.radioTelemetryStatus || "Waiting for telemetry").toLowerCase()
        if (status.includes("stale")) return packets + " packets · stale"
        if (status.includes("receiving buoy telemetry")) return packets + " packets · receiving"
        if (status.includes("heartbeat")) return packets + " packets · heartbeat only"
        if (status.includes("waiting")) return packets + " packets · waiting"
        return packets + " packets · " + status
    }
    function sessionSummary() {
        if (String(s.telemetrySource || "").toUpperCase() === "ELRS" && buoyState) {
            if (!buoyState.recording) return "SESSION IDLE"
            if (s.radioAudioRecording === true) return "SESSION + AUDIO"
            if (s.radioAudioRecording === false) return "SESSION ONLY"
            return "SESSION RECORDING"
        }
        if (!s.wifiDetailTelemetryLive && !s.wifiSafetyTelemetryLive) return "SESSION STATE WAITING"
        const state = String(s.scienceExperimentState || "UNKNOWN").toUpperCase()
        if (state === "RECORDING") return s.audioRecordingEnabled ? "SESSION + AUDIO" : "SESSION ONLY"
        if (state === "STOPPED") return "SAVE PENDING"
        if (state === "IDLE") return "SESSION IDLE"
        return "SESSION STATE WAITING"
    }
    function sessionActive() {
        const status = sessionSummary()
        return status === "SESSION + AUDIO" || status === "SESSION ONLY" || status === "SESSION RECORDING"
    }
    function sessionColor() {
        if (sessionActive()) return "#58d7a5"
        if (sessionSummary() === "SAVE PENDING") return "#f1bd68"
        return "#91a8b9"
    }

    radius: 12
    color: tileMouse.containsMouse ? "#1b3043" : "#111c28"
    border.color: tileMouse.containsMouse ? "#50aaff" : "#395166"
    clip: true

    Text {
        id: titleText
        x: 14; y: 12; width: root.title === "Communications" ? parent.width - 150 : parent.width - 28
        text: root.title
        color: "#80bfff"; font.pixelSize: 15; font.bold: true
    }
    Rectangle {
        visible: root.title === "Communications"
        x: parent.width - width - 14; y: 8; width: 122; height: 31; radius: 16
        color: root.linkQualityBackground()
        border.color: root.linkQualityColor()
        Text {
            anchors.centerIn: parent
            text: root.linkQualitySummary()
            color: root.linkQualityColor()
            font.pixelSize: 12; font.bold: true
        }
    }
    Text {
        visible: root.title === "Power"
        x: 14; y: 40
        text: (Number.isFinite(root.batteryLevel) ? Math.round(root.batteryLevel) + "%" : "—")
              + "   " + (Number.isFinite(root.batteryVoltage) ? root.batteryVoltage.toFixed(2) + " V" : "— V")
        color: "#e6ecf2"; font.pixelSize: 22; font.bold: true
    }
    Text {
        visible: root.title === "Power"
        x: parent.width - 110; y: 42; width: 94; horizontalAlignment: Text.AlignRight
        text: Number.isFinite(root.currentAmps)
              ? (root.currentAmps > 0.05 ? "↓" : root.currentAmps < -0.05 ? "↑" : "·").repeat(Math.abs(root.currentAmps) < 0.75 ? 1 : Math.abs(root.currentAmps) < 2.5 ? 2 : 3)
                + "  " + Math.abs(root.currentAmps).toFixed(1) + " A"
              : "Current —"
        color: root.currentAmps > 0.05 ? "#f2bb74" : "#78ddb2"; font.pixelSize: 16; font.bold: true
    }
    Text {
        visible: root.title === "Power"
        x: 14; y: 68; text: "BATTERY VOLTAGE OVER TIME"
        color: "#829daf"; font.pixelSize: 9; font.bold: true
    }
    Item {
        visible: root.title === "Communications"
        x: 14; y: 45; width: parent.width - 28; height: parent.height - 55
        Rectangle { x: parent.width / 2; y: 2; width: 1; height: parent.height - 4; color: "#304559" }
        Column {
            x: 0; y: 2; width: (parent.width - 16) / 2; spacing: 6
            Text { text: "BUOY STATE"; color: "#829aad"; font.pixelSize: 8; font.bold: true }
            Text { width: parent.width; text: root.buoyArmSummary() + " · " + root.buoyModeSummary().toUpperCase(); color: root.buoyArmSummary() === "ARMED" ? "#f1bd68" : "#e6ecf2"; font.pixelSize: 11; font.bold: true; elide: Text.ElideRight }
            Text { width: parent.width; text: root.controlSummary(); color: "#dce8ef"; font.pixelSize: 9; elide: Text.ElideRight }
            Text { width: parent.width; text: root.routeSummary(); color: "#9fd6ef"; font.pixelSize: 9; elide: Text.ElideRight }
        }
        Column {
            x: parent.width / 2 + 10; y: 2; width: (parent.width - 16) / 2; spacing: 6
            Text { text: "TX12 & TELEMETRY"; color: "#829aad"; font.pixelSize: 8; font.bold: true }
            Text { width: parent.width; text: root.switchSummary(5, "SE"); color: "#dce8ef"; font.pixelSize: 9; elide: Text.ElideRight }
            Text { width: parent.width; text: root.switchSummary(7, "SB"); color: "#dce8ef"; font.pixelSize: 9; elide: Text.ElideRight }
            Text { width: parent.width; text: root.telemetrySummary(); color: root.buoyState ? "#58d7a5" : "#f1bd68"; font.pixelSize: 9; elide: Text.ElideRight }
        }
        Rectangle {
            x: 2
            y: 80
            width: parent.width - 4
            height: 19
            radius: 9
            color: root.sessionActive() ? "#17382f"
                : root.sessionSummary() === "SAVE PENDING" ? "#3b3021" : "#233342"
            border.color: root.sessionColor()
            Text {
                anchors.fill: parent
                anchors.leftMargin: 5
                anchors.rightMargin: 5
                text: root.sessionSummary()
                color: root.sessionColor()
                font.pixelSize: 8
                font.bold: true
                verticalAlignment: Text.AlignVCenter
                horizontalAlignment: Text.AlignHCenter
                elide: Text.ElideRight
            }
        }
    }
    Canvas {
        id: chart
        visible: root.title === "Navigation" || root.title === "Power"
        x: 14; y: root.title === "Power" ? 82 : 42
        width: parent.width - 28
        height: root.title === "Power" ? 64 : 104
        onWidthChanged: requestPaint()
        onHeightChanged: requestPaint()
        Component.onCompleted: requestPaint()
        onPaint: {
            const ctx = getContext("2d")
            ctx.reset()
            ctx.clearRect(0, 0, width, height)
            if (root.title === "Power") {
                const points = root.powerSamples
                const values = points.map(point => Number(point.v)).filter(value => Number.isFinite(value))
                const floor = Math.min(...values), ceiling = Math.max(...values)
                const low = floor - 0.1, high = ceiling + 0.1
                const left = 2, right = width - 2, top = 4, bottom = height - 4
                ctx.strokeStyle = "#2b4558"; ctx.lineWidth = 1
                ctx.beginPath(); ctx.moveTo(left, bottom); ctx.lineTo(right, bottom); ctx.stroke()
                if (root.powerTrendAvailable && values.length > 1) {
                    const first = Number(points[0].t), last = Number(points[points.length - 1].t)
                    const timeSpan = last - first
                    ctx.beginPath()
                    let started = false
                    for (const point of points) {
                        const t = Number(point.t), v = Number(point.v)
                        if (!Number.isFinite(t) || !Number.isFinite(v)) continue
                        const x = left + (t - first) / timeSpan * (right - left)
                        const y = bottom - (v - low) / (high - low) * (bottom - top)
                        if (!started) { ctx.moveTo(x, y); started = true } else ctx.lineTo(x, y)
                    }
                    ctx.strokeStyle = "#75baff"; ctx.lineWidth = 3; ctx.lineJoin = "round"; ctx.stroke()
                } else {
                    ctx.fillStyle = "#91aabd"; ctx.font = "11px sans-serif"
                    ctx.fillText("Voltage trend collecting", left + 5, (top + bottom) / 2)
                }
            } else if (root.title === "Navigation") {
                const cx = 48, cy = height / 2, radius = Math.max(16, Math.min(height / 2 - 5, 42))
                ctx.fillStyle = "#0b1722"; ctx.beginPath(); ctx.arc(cx, cy, radius, 0, Math.PI * 2); ctx.fill()
                ctx.strokeStyle = "#284252"; ctx.lineWidth = 2
                ctx.beginPath(); ctx.arc(cx, cy, radius, 0, Math.PI * 2); ctx.stroke()
                const heading = root.headingAvailable ? root.heading : 0
                ctx.save(); ctx.translate(cx, cy); ctx.rotate(-heading * Math.PI / 180)
                ctx.textAlign = "center"; ctx.textBaseline = "middle"
                for (let degree = 0; degree < 360; degree += 15) {
                    const angle = (degree - 90) * Math.PI / 180
                    const cardinal = degree % 90 === 0, major = degree % 30 === 0
                    ctx.strokeStyle = cardinal ? "#b5d9e5" : major ? "#7296a7" : "#476374"
                    ctx.lineWidth = cardinal ? 2 : 1
                    ctx.beginPath()
                    ctx.moveTo(Math.cos(angle) * (radius - (major ? 10 : 5)), Math.sin(angle) * (radius - (major ? 10 : 5)))
                    ctx.lineTo(Math.cos(angle) * (radius - 2), Math.sin(angle) * (radius - 2))
                    ctx.stroke()
                    if (cardinal) {
                        ctx.save(); ctx.translate(Math.cos(angle) * (radius - 19), Math.sin(angle) * (radius - 19)); ctx.rotate(heading * Math.PI / 180)
                        ctx.fillStyle = degree === 0 ? "#78dadd" : "#c8d9e2"
                        ctx.font = "bold 10px sans-serif"
                        ctx.fillText(degree === 0 ? "N" : degree === 90 ? "E" : degree === 180 ? "S" : "W", 0, 0)
                        ctx.restore()
                    }
                }
                ctx.restore()
                ctx.fillStyle = "#6ce0de"; ctx.beginPath(); ctx.moveTo(cx, cy - radius - 4); ctx.lineTo(cx - 5, cy - radius + 5); ctx.lineTo(cx + 5, cy - radius + 5); ctx.closePath(); ctx.fill()
                if (root.headingAvailable && root.courseAvailable) {
                    const relative = (Number(root.s.navigationGpsCourseDeg) - heading - 90) * Math.PI / 180
                    ctx.fillStyle = "#75baff"; ctx.beginPath(); ctx.arc(cx + Math.cos(relative) * (radius - 2), cy + Math.sin(relative) * (radius - 2), 4, 0, Math.PI * 2); ctx.fill()
                }
                ctx.fillStyle = "#d9ecf5"; ctx.beginPath(); ctx.arc(cx, cy, 2.5, 0, Math.PI * 2); ctx.fill()
            }
        }
    }
    Column {
        visible: root.title === "Navigation"
        x: chart.x + 106; y: chart.y + 5
        width: chart.width - 106; spacing: 8
        Text { width: parent.width; text: root.headingAvailable ? root.heading.toFixed(0) + "° heading" : "Heading unavailable"; color: "#e6ecf2"; font.pixelSize: 14; font.bold: true; elide: Text.ElideRight }
        Text { width: parent.width; text: root.s.navigationGpsSpeedAvailable ? Number(root.s.navigationGpsSpeedMps).toFixed(2) + " m/s ground speed" : "GPS speed unavailable"; color: "#a8c9dd"; font.pixelSize: 11; elide: Text.ElideRight }
        Text { text: root.s.currentLocation && root.s.currentLocation !== "Unknown" ? root.s.currentLocation : "Waiting for GPS position"; color: "#a8c0d0"; font.pixelSize: 10; wrapMode: Text.Wrap; width: parent.width; maximumLineCount: 2; elide: Text.ElideRight }
    }

    Item {
        visible: root.title === "Propulsion"
        x: 14; y: 42; width: parent.width - 28; height: 109
        Repeater {
            model: [
                { label: "Left", key: "left" },
                { label: "Right", key: "right" },
                { label: "Rear", key: "rear" }
            ]
            delegate: Item {
                required property int index
                required property var modelData
                x: 0; y: index * 29; width: parent.width; height: 27
                Text { x: 0; y: 1; width: 40; text: modelData.label; color: "#a9bfd0"; font.pixelSize: 11 }
                Rectangle { x: 43; y: 6; width: Math.max(20, parent.width - 128); height: 9; radius: 4; color: "#20384a" }
                Rectangle {
                    x: 43; y: 6; height: 9; radius: 4; color: "#58baff"
                    width: root.motorEstimateAvailable ? Math.max(0, parent.width - 128) * (Number(root.motorEnergy[modelData.key]) || 0) / root.motorEnergyMax : 0
                }
                Text {
                    x: parent.width - 83; y: 1; width: 83; horizontalAlignment: Text.AlignRight
                    text: root.motorEstimateAvailable ? (Number(root.motorEnergy[modelData.key]) || 0).toFixed(2) + " Wh" : "—"
                    color: "#dce8f0"; font.pixelSize: 11; font.bold: true
                }
            }
        }
        Text {
            x: 0; y: 90; width: parent.width; elide: Text.ElideRight
            text: root.motorEstimateAvailable ? "Estimated since station telemetry began" : "Waiting for motor energy estimate"
            color: "#849fb1"; font.pixelSize: 10
        }
    }

    Item {
        visible: root.title === "Instruments"
        x: 16; y: 45; width: parent.width - 32; height: 106
        Item {
            anchors.fill: parent
            Repeater {
                model: [
                    { label: "WATER TEMP", value: root.s.waterTemperature || "—", color: "#66c9f4" },
                    { label: "DEPTH", value: root.s.currentDepth || "—", color: "#65b8ff" },
                    { label: "CONDUCTIVITY", value: root.s.conductivityVoltage || "—", color: "#78ddb2" },
                    { label: "TURBIDITY", value: root.s.turbidityVoltage || "—", color: "#f2bb74" }
                ]
                delegate: Column {
                    required property int index
                    required property var modelData
                    x: (index % 2) * (parent.width + 12) / 2
                    y: Math.floor(index / 2) * 52
                    width: (parent.width - 12) / 2
                    height: 50
                    spacing: 5
                    Text { text: modelData.label; color: "#7f9aaf"; font.pixelSize: 9; font.bold: true; elide: Text.ElideRight; width: parent.width }
                    Text { text: modelData.value; color: modelData.color; font.pixelSize: 16; font.bold: true; elide: Text.ElideRight; width: parent.width }
                }
            }
        }
    }

    Item {
        visible: root.title === "SD Card"
        x: 16; y: 47; width: parent.width - 32; height: 103
        Text { text: root.cardTotal > 0 ? Math.round(root.cardFraction * 100) + "% used" : "Capacity unavailable"; color: "#e6ecf2"; font.pixelSize: root.cardTotal > 0 ? 23 : 16; font.bold: true }
        Rectangle { x: 0; y: 43; width: parent.width; height: 14; radius: 7; color: "#20384a" }
        Rectangle { x: 0; y: 43; width: parent.width * root.cardFraction; height: 14; radius: 7; color: "#aa88ec" }
        Text { x: 0; y: 72; width: parent.width; text: root.cardTotal > 0 ? sizeLabel(root.cardUsed) + " used  ·  " + sizeLabel(root.cardFree) + " free" : (root.s.sdBusy ? "Reading SD card…" : "Connect buoy to read capacity")
            color: "#9eb5c7"; font.pixelSize: 11; elide: Text.ElideRight }
    }

    MouseArea {
        id: tileMouse
        anchors.fill: parent
        hoverEnabled: true
        cursorShape: Qt.PointingHandCursor
        onClicked: if (root.backendObject) root.backendObject.setDashboardTab(root.tab)
    }
}
