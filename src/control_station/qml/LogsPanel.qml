pragma ComponentBehavior: Bound

import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Item {
    id: root
    property var backendObject
    property var backendState: ({})
    property bool active: true
    property string kindFilter: "all"
    property string searchFilter: ""
    property bool paused: false
    property bool followLive: true
    property bool captureOptionsVisible: false
    property var frozenEntries: []
    property var expandedRows: ({})
    property int clearThroughSequence: 0

    readonly property var incomingEntries: backendObject && active ? backendObject.linkLog : []
    readonly property var displayedEntries: buildEntries()
    readonly property int newCount: paused && incomingEntries.length
        ? Math.max(0, Number(incomingEntries[incomingEntries.length - 1].sequence || 0)
            - Number(frozenEntries.length ? frozenEntries[frozenEntries.length - 1].sequence : clearThroughSequence)) : 0

    function pauseLive() {
        if (paused) return
        const offset = logList.contentY
        frozenEntries = (incomingEntries || []).slice()
        paused = true
        followLive = false
        Qt.callLater(function() {
            logList.contentY = Math.max(0, Math.min(offset, logList.contentHeight - logList.height))
        })
    }

    function resumeLive() {
        paused = false
        followLive = true
        Qt.callLater(function() { logList.positionViewAtEnd() })
    }

    function clearView() {
        const entries = incomingEntries || []
        if (entries.length) clearThroughSequence = Number(entries[entries.length - 1].sequence || 0)
        frozenEntries = []
        paused = false
        followLive = true
    }

    function countFor(kind) {
        const entries = paused ? frozenEntries : incomingEntries
        let count = 0
        for (const entry of entries || []) {
            if (Number(entry.sequence || 0) <= clearThroughSequence) continue
            if (!backendState.elrsPacketLogging && String(entry.message || "").startsWith("ELRS MAVLink RX ")) continue
            if (kind === "all" || (kind === "errors" ? entry.severity === "error" : entry.kind === kind)) count++
        }
        return count
    }

    function repeatKey(entry) {
        if (entry.severity !== "info") return ""
        const message = String(entry.message || "").toUpperCase()
        const match = message.match(/(?:TEL STATUS (?:ELRS_CHANNELS|ELRS_LINK|TELEMETRY_MODE)|ELRS MAVLINK RX|TX PING|RX ACK PING)/)
        return match ? String(entry.kind) + "|" + match[0] : ""
    }

    function buildEntries() {
        const source = paused ? frozenEntries : incomingEntries
        const query = searchFilter.trim().toLowerCase()
        const rows = []
        const recentGroups = ({})
        for (const entry of source || []) {
            if (Number(entry.sequence || 0) <= clearThroughSequence) continue
            if (!backendState.elrsPacketLogging && String(entry.message || "").startsWith("ELRS MAVLink RX ")) continue
            if (kindFilter !== "all" && (kindFilter === "errors" ? entry.severity !== "error" : entry.kind !== kindFilter)) continue
            if (query && (String(entry.utc || "") + " " + String(entry.source || "") + " "
                + String(entry.message || "") + " " + String(entry.interpretation || "")).toLowerCase().indexOf(query) < 0) continue
            const key = repeatKey(entry)
            const previous = key ? recentGroups[key] : null
            if (previous && Number(entry.epochMs || 0) - previous.lastMs <= 10000) {
                previous.count++
                previous.lastMs = Number(entry.epochMs || 0)
                previous.entry = entry
                const index = rows.indexOf(previous)
                if (index >= 0) rows.splice(index, 1)
                rows.push(previous)
            } else {
                const group = { entry: entry, count: 1, lastMs: Number(entry.epochMs || 0) }
                rows.push(group)
                if (key) recentGroups[key] = group
            }
        }
        return rows
    }

    function kindColor(kind) {
        if (kind === "errors") return "#f07878"
        if (kind === "elrs") return "#7abfff"
        if (kind === "commands") return "#72d4cf"
        return "#a9c0d0"
    }

    function severityColor(severity) {
        if (severity === "error") return "#f07878"
        if (severity === "warning") return "#f1bd68"
        return "#7b93a6"
    }

    component PanelButton: Button {
        id: panelButton
        property bool primary: false
        property bool selected: false
        implicitHeight: 34
        implicitWidth: Math.max(45, buttonLabel.implicitWidth + 20)
        background: Rectangle {
            radius: 8
            color: !panelButton.enabled ? "#172431" : panelButton.primary || panelButton.selected ? "#175662" : panelButton.hovered ? "#263c4d" : "#1c2d3b"
            border.color: !panelButton.enabled ? "#263643" : panelButton.primary || panelButton.selected ? "#3c9ca7" : "#385064"
        }
        contentItem: Text {
            id: buttonLabel
            text: panelButton.text
            color: !panelButton.enabled ? "#657b8b" : panelButton.primary || panelButton.selected ? "#e8ffff" : "#dbe8ef"
            font.pixelSize: 11; font.bold: true
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
        }
    }

    Rectangle {
        anchors.fill: parent
        radius: 12; color: "#0b1119"; border.color: "#395166"

        ColumnLayout {
            anchors.fill: parent; anchors.margins: 14; spacing: 10

            RowLayout {
                Layout.fillWidth: true; spacing: 10
                ColumnLayout {
                    Layout.fillWidth: true; spacing: 2
                    Text { text: "Link log"; color: "#e6ecf2"; font.pixelSize: 21; font.bold: true }
                    Text { text: "Wi-Fi, ELRS, commands, and faults"; color: "#91a8b9"; font.pixelSize: 11 }
                }
                Rectangle { Layout.preferredWidth: 7; Layout.preferredHeight: 7; radius: 4; color: root.paused ? "#f1bd68" : "#58d7a5" }
                Text { text: root.paused ? "PAUSED" : "LIVE"; color: root.paused ? "#f1bd68" : "#9ce8c5"; font.pixelSize: 10; font.bold: true }
                PanelButton { text: root.paused ? "Follow latest" + (root.newCount ? " (" + root.newCount + ")" : "") : "Pause live"; primary: root.paused; onClicked: root.paused ? root.resumeLive() : root.pauseLive() }
                PanelButton { text: "Clear view"; onClicked: root.clearView() }
                PanelButton { text: "Export full log"; enabled: Boolean(root.backendObject); onClicked: root.backendObject.exportLinkLog() }
            }

            RowLayout {
                Layout.fillWidth: true; spacing: 6
                Repeater {
                    model: [
                        { key: "all", label: "All" },
                        { key: "wifi", label: "Wi-Fi" },
                        { key: "elrs", label: "ELRS" },
                        { key: "commands", label: "Commands" },
                        { key: "errors", label: "Errors" }
                    ]
                    delegate: PanelButton {
                        required property var modelData
                        text: modelData.label + "  " + root.countFor(modelData.key)
                        selected: root.kindFilter === modelData.key
                        onClicked: root.kindFilter = modelData.key
                    }
                }
                Item { Layout.fillWidth: true }
                TextField {
                    id: searchInput
                    Layout.preferredWidth: Math.min(320, Math.max(180, parent.width * 0.27))
                    implicitHeight: 34
                    placeholderText: "Search time, source, or message"
                    color: "#e6ecf2"; font.pixelSize: 11
                    onTextChanged: root.searchFilter = text
                    background: Rectangle { radius: 8; color: "#111b27"; border.color: searchInput.activeFocus ? "#3c9ca7" : "#385064" }
                }
            }

            RowLayout {
                Layout.fillWidth: true; spacing: 8
                PanelButton { text: root.captureOptionsVisible ? "Hide capture options" : "Capture options"; onClicked: root.captureOptionsVisible = !root.captureOptionsVisible }
                StyledCheckBox {
                    visible: root.captureOptionsVisible
                    text: "Record raw ELRS packets"
                    checked: Boolean(root.backendState.elrsPacketLogging)
                    onToggled: if (root.backendObject) root.backendObject.setElrsPacketLogging(checked)
                }
                Item { Layout.fillWidth: true }
                Text { text: root.paused && root.newCount ? root.newCount + " new events" : ""; color: "#f1bd68"; font.pixelSize: 11 }
            }

            ListView {
                id: logList
                Layout.fillWidth: true; Layout.fillHeight: true
                clip: true; spacing: 5
                model: root.displayedEntries
                boundsBehavior: Flickable.StopAtBounds
                ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }
                onCountChanged: if (!root.paused && root.followLive) Qt.callLater(function() { logList.positionViewAtEnd() })
                onMovementStarted: root.followLive = false
                onMovementEnded: if (!root.paused && !root.followLive) root.pauseLive()

                delegate: Rectangle {
                    id: logRow
                    required property var modelData
                    required property int index
                    readonly property var event: modelData.entry
                    readonly property bool expanded: Boolean(root.expandedRows[String(event.sequence)])
                    width: logList.width
                    implicitHeight: rowContents.implicitHeight + 18
                    radius: 9; color: index % 2 ? "#111c28" : "#101925"
                    border.color: expanded ? "#527285" : "#243b4d"

                    Rectangle { width: 4; height: parent.height - 12; x: 0; y: 6; radius: 2; color: root.severityColor(logRow.event.severity) }
                    ColumnLayout {
                        id: rowContents
                        anchors.left: parent.left; anchors.right: parent.right
                        anchors.top: parent.top; anchors.margins: 9
                        anchors.leftMargin: 13; anchors.rightMargin: 12
                        spacing: 7
                        RowLayout {
                            Layout.fillWidth: true; spacing: 10
                            Text { text: logRow.event.timestamp || ""; color: "#8fa6b8"; font.family: "monospace"; font.pixelSize: 10; Layout.preferredWidth: 95 }
                            Text { text: logRow.event.source || "Station"; color: root.kindColor(logRow.event.kind); font.pixelSize: 10; font.bold: true; Layout.preferredWidth: 68; elide: Text.ElideRight }
                            Text { text: String(logRow.event.severity || "info").toUpperCase(); color: root.severityColor(logRow.event.severity); font.pixelSize: 9; font.bold: true; Layout.preferredWidth: 58 }
                            Text { text: logRow.event.interpretation || logRow.event.message || ""; color: "#e6ecf2"; font.pixelSize: 12; Layout.fillWidth: true; elide: Text.ElideRight }
                            Text { visible: logRow.modelData.count > 1; text: "×" + logRow.modelData.count; color: "#9ce8e3"; font.pixelSize: 11; font.bold: true }
                            Text { text: logRow.expanded ? "▴" : "▾"; color: "#91a8b9"; font.pixelSize: 13 }
                        }
                        ColumnLayout {
                            visible: logRow.expanded
                            Layout.fillWidth: true; spacing: 4
                            Text { text: logRow.event.utc || ""; color: "#91a8b9"; font.family: "monospace"; font.pixelSize: 10 }
                            Text { text: "RAW  " + String(logRow.event.message || ""); color: "#9eb5c7"; font.family: "monospace"; font.pixelSize: 10; wrapMode: Text.WrapAnywhere; Layout.fillWidth: true }
                            Text { visible: logRow.modelData.count > 1; text: "Grouped " + logRow.modelData.count + " repeated updates within 10 seconds"; color: "#7d96a8"; font.pixelSize: 10 }
                        }
                    }
                    TapHandler {
                        onTapped: {
                            const next = Object.assign({}, root.expandedRows)
                            const key = String(logRow.event.sequence)
                            next[key] = !next[key]
                            root.expandedRows = next
                        }
                    }
                }

                Text {
                    anchors.centerIn: parent
                    visible: logList.count === 0
                    text: root.countFor("all") === 0 ? "Waiting for link events" : "No events match these filters"
                    color: "#829aad"; font.pixelSize: 12
                }
            }

            Text {
                Layout.fillWidth: true
                text: backendState.linkLogExportStatus || "Saved session events remain available after Clear view; this window shows the latest 360 link events."
                color: "#829aad"; font.pixelSize: 10; elide: Text.ElideMiddle
            }
        }
    }
}
