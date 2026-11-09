import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Item {
    id: root
    property var backendObject
    property var backendState: ({})
    property var samples: backendState.powerHistory || []
    property int intervalSeconds: Number(backendState.powerHistoryIntervalSeconds || 300)
    property int windowSeconds: Number(backendState.powerHistoryWindowSeconds || 86400)
    property string selectedDay: ""
    property real lowVoltage: 10.2
    property var dayKeys: availableDays()
    property var daySamples: samples.filter(point => String(Math.floor(Number(point.t) / 86400)) === selectedDay)
    property var displaySamples: intervalSeconds === 300 ? daySamples : samples
    property var dayStats: summarizeDay()

    onDayKeysChanged: {
        if (dayKeys.length && dayKeys.indexOf(selectedDay) < 0)
            selectedDay = dayKeys[dayKeys.length - 1]
    }
    onDisplaySamplesChanged: chart.requestPaint()
    onIntervalSecondsChanged: chart.requestPaint()
    onLowVoltageChanged: chart.requestPaint()

    function availableDays() {
        const seen = ({})
        const keys = []
        for (const point of samples) {
            const key = String(Math.floor(Number(point.t) / 86400))
            if (!seen[key]) { seen[key] = true; keys.push(key) }
        }
        return keys
    }

    function dayLabel(key) {
        if (key === "") return "No day"
        return new Date(Number(key) * 86400000).toISOString().slice(0, 10) + " UTC"
    }

    function timeLabel(timestamp) {
        return new Date(Number(timestamp) * 1000).toISOString().slice(11, 16)
    }

    function summarizeDay() {
        const result = { min: NaN, max: NaN, minAt: 0, maxAt: 0, lowBins: 0, firstLow: 0 }
        for (const point of displaySamples) {
            const lo = Number(point.vmin), hi = Number(point.vmax)
            if (!Number.isFinite(result.min) || lo < result.min) {
                result.min = lo; result.minAt = Number(point.tmin)
            }
            if (!Number.isFinite(result.max) || hi > result.max) {
                result.max = hi; result.maxAt = Number(point.tmax)
            }
            if (lo < lowVoltage) {
                result.lowBins++
                if (!result.firstLow) result.firstLow = Number(point.tmin)
            }
        }
        return result
    }

    function currentText() {
        if (backendState.telemetrySource === "ELRS") return "Direction unavailable over ELRS"
        const value = Number.parseFloat(String(backendState.currentDraw || ""))
        if (!Number.isFinite(value)) return "Waiting for current telemetry"
        if (value > 0.05) return "Discharging " + value.toFixed(2) + " A"
        if (value < -0.05) return "Estimated charging " + Math.abs(value).toFixed(2) + " A"
        return "Current near zero"
    }

    function drawChart(ctx, width, height) {
        ctx.clearRect(0, 0, width, height)
        ctx.fillStyle = "#0e1a26"
        ctx.fillRect(0, 0, width, height)
        const left = 52, right = width - 18, top = 22
        const batteryBottom = Math.max(top + 75, height * 0.66)
        const currentTop = batteryBottom + 27
        const currentBottom = height - 30
        const plotWidth = Math.max(1, right - left)
        const slotWidth = plotWidth / (windowSeconds / intervalSeconds)
        const lastSample = displaySamples.length ? Number(displaySamples[displaySamples.length - 1].t) : Math.floor(Date.now() / 1000)
        const windowStart = intervalSeconds === 300 ? Number(selectedDay || 0) * 86400
            : Math.floor(lastSample / intervalSeconds) * intervalSeconds + intervalSeconds - windowSeconds
        const stats = dayStats
        const axisMin = Math.min(9.6, Number.isFinite(stats.min) ? Math.floor((stats.min - 0.1) * 10) / 10 : 9.6)
        const axisMax = Math.max(12.6, Number.isFinite(stats.max) ? Math.ceil((stats.max + 0.1) * 10) / 10 : 12.6)
        const yFor = v => batteryBottom - (v - axisMin) / (axisMax - axisMin) * (batteryBottom - top)
        const xFor = t => left + (Number(t) - windowStart) / intervalSeconds * slotWidth
        const middle = (currentTop + currentBottom) / 2

        for (let tick = 0; tick <= 6; tick++) {
            const x = left + tick / 6 * plotWidth
            ctx.strokeStyle = "#2a4053"; ctx.lineWidth = 1
            ctx.beginPath(); ctx.moveTo(x, top); ctx.lineTo(x, currentBottom); ctx.stroke()
            ctx.fillStyle = "#91aabd"; ctx.font = "10px sans-serif"
            const label = intervalSeconds === 300 && tick === 6 ? "24:00" :
                          root.timeLabel(windowStart + tick / 6 * windowSeconds)
            ctx.fillText(label, Math.min(right - 28, x - 13), height - 10)
        }
        for (const value of [axisMin, 10.2, 11.4, axisMax]) {
            if (value < axisMin || value > axisMax) continue
            const y = yFor(value)
            ctx.strokeStyle = "#2a4053"; ctx.lineWidth = 1
            ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(right, y); ctx.stroke()
            ctx.fillStyle = "#91aabd"; ctx.font = "10px sans-serif"
            ctx.fillText(value.toFixed(1), 10, y + 3)
        }
        const thresholdY = yFor(lowVoltage)
        ctx.strokeStyle = "#ef6868"; ctx.lineWidth = 1.5
        ctx.beginPath(); ctx.moveTo(left, thresholdY); ctx.lineTo(right, thresholdY); ctx.stroke()
        ctx.fillStyle = "#ef8585"
        ctx.fillText("LOW " + lowVoltage.toFixed(1) + " V", Math.max(left, right - 70), thresholdY - 5)
        ctx.fillStyle = "#91aabd"
        ctx.font = "bold 10px sans-serif"
        ctx.fillText("NET CURRENT", left + 8, currentTop - 8)
        ctx.strokeStyle = "#718a9b"
        ctx.beginPath(); ctx.moveTo(left, middle); ctx.lineTo(right, middle); ctx.stroke()

        let currentScale = 0.5
        for (const point of displaySamples) {
            const amps = Number(point.a)
            if (Number.isFinite(amps)) currentScale = Math.max(currentScale, Math.abs(amps))
        }
        currentScale *= 1.1
        const currentHeight = Math.max(1, (currentBottom - currentTop) / 2 - 2)
        const inWindow = displaySamples.filter(point => Number(point.t) >= windowStart && Number(point.t) <= windowStart + windowSeconds)
        const voltageFill = ctx.createLinearGradient(0, top, 0, batteryBottom)
        voltageFill.addColorStop(0, "rgba(117, 186, 255, 0.38)")
        voltageFill.addColorStop(1, "rgba(117, 186, 255, 0.05)")
        for (let start = 0; start < inWindow.length;) {
            let end = start + 1
            while (end < inWindow.length && Number(inWindow[end].t) - Number(inWindow[end - 1].t) <= intervalSeconds * 1.5) end++
            if (end - start > 1) {
                ctx.beginPath()
                ctx.moveTo(xFor(inWindow[start].t), batteryBottom)
                for (let i = start; i < end; i++) ctx.lineTo(xFor(inWindow[i].t), yFor(Number(inWindow[i].v)))
                ctx.lineTo(xFor(inWindow[end - 1].t), batteryBottom)
                ctx.closePath(); ctx.fillStyle = voltageFill; ctx.fill()
                ctx.beginPath()
                for (let i = start; i < end; i++) {
                    const x = xFor(inWindow[i].t), y = yFor(Number(inWindow[i].v))
                    if (i === start) ctx.moveTo(x, y); else ctx.lineTo(x, y)
                }
                ctx.strokeStyle = "rgba(117, 186, 255, 0.20)"; ctx.lineWidth = 8; ctx.stroke()
                ctx.strokeStyle = "#75baff"; ctx.lineWidth = 3.5; ctx.stroke()
            } else {
                ctx.fillStyle = "#75baff"; ctx.beginPath(); ctx.arc(xFor(inWindow[start].t), yFor(Number(inWindow[start].v)), 4, 0, Math.PI * 2); ctx.fill()
            }
            start = end
        }
        const currentY = amps => middle - amps / currentScale * currentHeight
        for (const positive of [true, false]) {
            ctx.beginPath()
            for (let i = 1; i < inWindow.length; i++) {
                const first = inWindow[i - 1], second = inWindow[i]
                if (Number(second.t) - Number(first.t) > intervalSeconds * 1.5) continue
                const firstAmp = Number(first.a), secondAmp = Number(second.a)
                if (!Number.isFinite(firstAmp) || !Number.isFinite(secondAmp)) continue
                const x1 = xFor(first.t), x2 = xFor(second.t)
                const a1 = positive ? Math.max(0, firstAmp) : Math.min(0, firstAmp)
                const a2 = positive ? Math.max(0, secondAmp) : Math.min(0, secondAmp)
                if (a1 === 0 && a2 === 0) continue
                const zeroX = firstAmp * secondAmp < 0
                    ? x1 + (x2 - x1) * Math.abs(firstAmp) / (Math.abs(firstAmp) + Math.abs(secondAmp)) : NaN
                const startX = a1 === 0 && Number.isFinite(zeroX) ? zeroX : x1
                const endX = a2 === 0 && Number.isFinite(zeroX) ? zeroX : x2
                ctx.moveTo(startX, middle)
                ctx.lineTo(startX, currentY(a1))
                ctx.lineTo(endX, currentY(a2))
                ctx.lineTo(endX, middle)
                ctx.closePath()
            }
            ctx.fillStyle = positive ? "rgba(242, 187, 116, 0.28)" : "rgba(120, 221, 178, 0.23)"
            ctx.fill()
        }
        for (let i = 1; i < inWindow.length; i++) {
            const first = inWindow[i - 1], second = inWindow[i]
            if (Number(second.t) - Number(first.t) > intervalSeconds * 1.5) continue
            const firstAmp = Number(first.a), secondAmp = Number(second.a)
            if (!Number.isFinite(firstAmp) || !Number.isFinite(secondAmp)) continue
            ctx.beginPath()
            ctx.moveTo(xFor(first.t), currentY(firstAmp))
            ctx.lineTo(xFor(second.t), currentY(secondAmp))
            ctx.strokeStyle = firstAmp + secondAmp < 0 ? "#78ddb2" : "#f2bb74"
            ctx.lineWidth = 3
            ctx.stroke()
        }

        if (Number.isFinite(stats.min)) {
            for (const marker of [
                { name: "MIN", value: stats.min, at: stats.minAt, color: "#ff8b8b" },
                { name: "MAX", value: stats.max, at: stats.maxAt, color: "#8bc5ff" }
            ]) {
                const x = xFor(marker.at), y = yFor(marker.value)
                ctx.fillStyle = marker.color
                ctx.beginPath(); ctx.arc(x, y, 4, 0, Math.PI * 2); ctx.fill()
                ctx.font = "bold 10px sans-serif"
                ctx.fillText(marker.name, Math.min(right - 25, x + 6), Math.max(top + 10, y - 7))
            }
        }
        if (!displaySamples.length) {
            ctx.fillStyle = "#91aabd"; ctx.font = "13px sans-serif"
            ctx.fillText("Waiting for battery telemetry in this view", left + 12, top + 35)
        }
    }


    function voltage() {
        const value = Number.parseFloat(String(backendState.batteryStatus || ""))
        return Number.isFinite(value) ? value : NaN
    }

    function amps() {
        const value = Number.parseFloat(String(backendState.currentDraw || ""))
        return Number.isFinite(value) ? value : NaN
    }

    function metric(value, digits, unit) {
        return Number.isFinite(value) ? value.toFixed(digits) + " " + unit : "—"
    }

    component PowerButton: Button {
        id: powerButton
        property bool selected: false
        implicitHeight: 34
        implicitWidth: Math.max(36, label.implicitWidth + 20)
        background: Rectangle {
            radius: 8
            color: powerButton.selected ? "#175662" : powerButton.hovered ? "#263c4d" : "#1c2d3b"
            border.color: powerButton.selected ? "#3c9ca7" : "#385064"
        }
        contentItem: Text {
            id: label
            text: powerButton.text
            color: powerButton.selected ? "#e8ffff" : "#dbe8ef"
            font.pixelSize: 11
            font.bold: true
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 18
        spacing: 12

        RowLayout {
            Layout.fillWidth: true
            ColumnLayout {
                Layout.fillWidth: true
                spacing: 2
                Text { text: "Power"; color: "#eef5f9"; font.pixelSize: 24; font.bold: true }
                Text { text: backendState.powerHistoryStatus || "Waiting for buoy telemetry"; color: "#91a9b9"; font.pixelSize: 11; elide: Text.ElideRight; Layout.fillWidth: true }
            }
            Rectangle {
                width: 100; height: 24; radius: 12
                color: root.samples.length ? "#173b34" : "#3a3030"
                border.color: root.samples.length ? "#346e58" : "#68504a"
                Text { anchors.centerIn: parent; text: root.samples.length ? "SAVED HISTORY" : "NO HISTORY"; color: root.samples.length ? "#78ddb2" : "#f2bb74"; font.pixelSize: 9; font.bold: true }
            }
        }

        GridLayout {
            Layout.fillWidth: true
            columns: 3
            columnSpacing: 12
            rowSpacing: 0
            Repeater {
                model: [
                    { label: "BATTERY VOLTAGE", value: root.metric(root.voltage(), 2, "V"), accent: "#75baff", detail: String(backendState.telemetrySource || "Buoy telemetry") },
                    { label: "BATTERY CURRENT", value: root.metric(root.amps(), 2, "A"), accent: "#f2bb74", detail: root.currentText() },
                    { label: "BATTERY POWER", value: root.metric(root.voltage() * Math.abs(root.amps()), 1, "W"), accent: "#78ddb2", detail: "Voltage × absolute current" }
                ]
                delegate: Rectangle {
                    required property var modelData
                    Layout.fillWidth: true
                    Layout.preferredHeight: 111
                    radius: 16
                    color: "#111e2a"
                    border.color: "#2a4051"
                    ColumnLayout {
                        anchors.fill: parent; anchors.margins: 16; spacing: 4
                        Text { text: modelData.label; color: modelData.accent; font.pixelSize: 10; font.bold: true; font.letterSpacing: 1.1 }
                        Text { text: modelData.value; color: "#eef5f9"; font.pixelSize: 28; font.bold: true }
                        Text { text: modelData.detail; color: "#91a9b9"; font.pixelSize: 10; elide: Text.ElideRight; Layout.fillWidth: true }
                    }
                }
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 56
            radius: 12; color: "#132432"; border.color: "#2a4051"
            RowLayout {
                anchors.fill: parent; anchors.margins: 10; spacing: 8
                Text { text: "HISTORY"; color: "#91a9b9"; font.pixelSize: 10; font.bold: true }
                Repeater {
                    model: [ { label: "24 h", seconds: 300 }, { label: "2 h", seconds: 30 }, { label: "30 min", seconds: 10 } ]
                    delegate: PowerButton {
                        required property var modelData
                        text: modelData.label
                        selected: root.intervalSeconds === modelData.seconds
                        onClicked: if (backendObject) backendObject.setPowerHistoryInterval(modelData.seconds)
                    }
                }
                ComboBox {
                    id: dayPicker
                    Layout.preferredWidth: 132
                    visible: root.intervalSeconds === 300
                    model: root.dayKeys.map(key => root.dayLabel(key))
                    currentIndex: Math.max(0, root.dayKeys.indexOf(root.selectedDay))
                    onActivated: root.selectedDay = root.dayKeys[currentIndex]
                    background: Rectangle { radius: 8; color: "#1c2d3b"; border.color: "#385064" }
                    contentItem: Text { text: dayPicker.displayText; color: "#dbe8ef"; font.pixelSize: 10; leftPadding: 8; verticalAlignment: Text.AlignVCenter; elide: Text.ElideRight }
                }
                Item { Layout.fillWidth: true }
                Text { text: "Low line"; color: "#91a9b9"; font.pixelSize: 10 }
                PowerButton { text: "−"; onClicked: root.lowVoltage = Math.max(9.6, Math.round((root.lowVoltage - 0.1) * 10) / 10) }
                Text { text: root.lowVoltage.toFixed(1) + " V"; color: "#eef5f9"; font.pixelSize: 11; font.bold: true; Layout.preferredWidth: 45; horizontalAlignment: Text.AlignHCenter }
                PowerButton { text: "+"; onClicked: root.lowVoltage = Math.min(12.6, Math.round((root.lowVoltage + 0.1) * 10) / 10) }
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumHeight: 250
            radius: 16; color: "#111e2a"; border.color: "#2a4051"
            ColumnLayout {
                anchors.fill: parent; anchors.margins: 14; spacing: 5
                RowLayout {
                    Layout.fillWidth: true
                    Text { text: root.intervalSeconds === 300 ? "BATTERY HISTORY · 24 HOURS" : "LIVE BATTERY HISTORY"; color: "#75baff"; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1.1; Layout.fillWidth: true }
                    Text { text: "Voltage above · current below"; color: "#91a9b9"; font.pixelSize: 10 }
                }
                Canvas {
                    id: chart
                    Layout.fillWidth: true; Layout.fillHeight: true
                    onWidthChanged: requestPaint()
                    onHeightChanged: requestPaint()
                    onPaint: root.drawChart(getContext("2d"), width, height)
                }
            }
        }

        GridLayout {
            Layout.fillWidth: true
            columns: 3
            columnSpacing: 12
            Repeater {
                model: [
                    { label: "MINIMUM", value: Number.isFinite(root.dayStats.min) ? root.dayStats.min.toFixed(2) + " V · " + root.timeLabel(root.dayStats.minAt) : "—", accent: "#f29b9b" },
                    { label: "MAXIMUM", value: Number.isFinite(root.dayStats.max) ? root.dayStats.max.toFixed(2) + " V · " + root.timeLabel(root.dayStats.maxAt) : "—", accent: "#75baff" },
                    { label: "BELOW LOW LINE", value: root.dayStats.lowBins ? root.dayStats.lowBins + " intervals · first " + root.timeLabel(root.dayStats.firstLow) : "None", accent: "#f2bb74" }
                ]
                delegate: Rectangle {
                    required property var modelData
                    Layout.fillWidth: true; Layout.preferredHeight: 66
                    radius: 12; color: "#111e2a"; border.color: "#2a4051"
                    ColumnLayout {
                        anchors.fill: parent; anchors.margins: 10; spacing: 2
                        Text { text: modelData.label; color: "#91a9b9"; font.pixelSize: 9; font.bold: true }
                        Text { text: modelData.value; color: modelData.accent; font.pixelSize: 13; font.bold: true; elide: Text.ElideRight; Layout.fillWidth: true }
                    }
                }
            }
        }

        Text {
            Layout.fillWidth: true
            text: "Shaded traces connect available readings. Gaps mean no data. During audio recording, power updates slow to about 30 seconds. The low line is advisory."
            color: "#91a9b9"; font.pixelSize: 10; wrapMode: Text.WordWrap
        }
    }
}
