pragma ComponentBehavior: Bound

import QtQuick
import QtQuick.Layouts

Item {
    id: root

    property var backendObject
    property var backendState: ({})
    property bool active: true
    property string categoryFilter: "all"
    property string searchFilter: ""

    readonly property var allEntries: backendObject && active ? backendObject.categorizedLog : []
    readonly property int systemCount: backendObject && active ? root.visibleLogCount(backendObject.systemLog) : 0
    readonly property int navigationCount: backendObject && active ? backendObject.navigationLog.length : 0
    readonly property int scientificCount: backendObject && active ? backendObject.scientificLog.length : 0

    function visibleLogCount(entries) {
        if (Boolean(root.backendState.elrsPacketLogging)) return entries.length
        let count = 0
        for (let i = 0; i < entries.length; ++i) {
            if (!String(entries[i].message || "").startsWith("ELRS MAVLink RX ")) count++
        }
        return count
    }

    function categoryTitle(category) {
        if (category === "navigation") return "NAVIGATION"
        if (category === "scientific") return "SCIENTIFIC"
        return "SYSTEM"
    }

    function categoryColor(category) {
        if (category === "navigation") return "#50aaff"
        if (category === "scientific") return "#56d6a9"
        if (category === "system") return "#ffad5c"
        return "#d8e2eb"
    }

    function categoryBackground(category, alternate) {
        if (category === "navigation") return alternate ? "#102238" : "#122942"
        if (category === "scientific") return alternate ? "#102a26" : "#12332d"
        return alternate ? "#2b2118" : "#35271a"
    }

    function filteredEntries() {
        const source = root.allEntries || []
        const query = root.searchFilter.trim().toLowerCase()
        const filtered = []
        for (let i = 0; i < source.length; ++i) {
            const entry = source[i]
            if (!Boolean(root.backendState.elrsPacketLogging) && String(entry.message || "").startsWith("ELRS MAVLink RX ")) continue
            if (root.categoryFilter !== "all" && entry.category !== root.categoryFilter) continue
            if (query.length > 0) {
                const searchable = String(entry.timestamp || "") + " " + String(entry.message || "") + " " + String(entry.interpretation || "")
                if (searchable.toLowerCase().indexOf(query) < 0) continue
            }
            filtered.push(entry)
        }
        return filtered
    }

    Rectangle {
        anchors.fill: parent
        radius: 12
        color: "#0b1119"
        border.color: "#395166"
        border.width: 1

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 14
            spacing: 10

            RowLayout {
                Layout.fillWidth: true
                spacing: 8

                Text {
                    text: "Event Logs"
                    color: "#e6ecf2"
                    font.pixelSize: 20
                    font.bold: true
                }

                Text {
                    Layout.fillWidth: true
                    text: "Chronological controller and telemetry activity"
                    color: "#829aad"
                    font.pixelSize: 11
                }

                Rectangle {
                    Layout.preferredWidth: 72
                    Layout.preferredHeight: 24
                    radius: 12
                    color: "#123226"
                    border.color: "#347b5f"

                    Text {
                        anchors.centerIn: parent
                        text: "● LIVE"
                        color: "#56d6a9"
                        font.pixelSize: 10
                        font.bold: true
                    }
                }
            }

            RowLayout {
                Layout.fillWidth: true
                spacing: 8

                Repeater {
                    model: [
                        { key: "all", label: "ALL EVENTS", count: root.visibleLogCount(root.allEntries), color: "#d8e2eb" },
                        { key: "system", label: "SYSTEM & MOTORS", count: root.systemCount, color: "#ffad5c" },
                        { key: "navigation", label: "NAVIGATION", count: root.navigationCount, color: "#50aaff" },
                        { key: "scientific", label: "SCIENTIFIC", count: root.scientificCount, color: "#56d6a9" }
                    ]

                    delegate: Rectangle {
                        id: statCard
                        required property var modelData

                        Layout.fillWidth: true
                        Layout.preferredHeight: 58
                        radius: 8
                        color: root.categoryFilter === statCard.modelData.key ? "#1d2b3a" : "#111b27"
                        border.color: root.categoryFilter === statCard.modelData.key ? statCard.modelData.color : "#2b3d4f"
                        border.width: root.categoryFilter === statCard.modelData.key ? 2 : 1

                        Column {
                            anchors.centerIn: parent
                            spacing: 2

                            Text {
                                anchors.horizontalCenter: parent.horizontalCenter
                                text: String(statCard.modelData.count)
                                color: statCard.modelData.color
                                font.pixelSize: 19
                                font.bold: true
                            }

                            Text {
                                anchors.horizontalCenter: parent.horizontalCenter
                                text: statCard.modelData.label
                                color: "#9eb5c7"
                                font.pixelSize: 9
                                font.bold: true
                            }
                        }

                        MouseArea {
                            anchors.fill: parent
                            cursorShape: Qt.PointingHandCursor
                            onClicked: root.categoryFilter = statCard.modelData.key
                        }
                    }
                }
            }

            RowLayout {
                Layout.fillWidth: true
                spacing: 8

                Text {
                    text: "FILTER"
                    color: "#7890a3"
                    font.pixelSize: 10
                    font.bold: true
                }

                Text {
                    text: root.categoryFilter === "all" ? "Showing all categories" : "Showing " + root.categoryFilter
                    color: root.categoryColor(root.categoryFilter)
                    font.pixelSize: 11
                }

                StyledCheckBox {
                    text: "Log ELRS packets"
                    checked: Boolean(root.backendState.elrsPacketLogging)
                    onToggled: if (root.backendObject) root.backendObject.setElrsPacketLogging(checked)
                }

                Item { Layout.fillWidth: true }

                Rectangle {
                    Layout.preferredWidth: 280
                    Layout.preferredHeight: 34
                    radius: 7
                    color: "#111b27"
                    border.color: searchInput.activeFocus ? "#50aaff" : "#34495d"

                    TextInput {
                        id: searchInput
                        anchors.fill: parent
                        anchors.leftMargin: 10
                        anchors.rightMargin: 10
                        verticalAlignment: TextInput.AlignVCenter
                        color: "#e6ecf2"
                        selectionColor: "#2f6f9f"
                        selectedTextColor: "#ffffff"
                        font.pixelSize: 12
                        clip: true
                        onTextChanged: root.searchFilter = text
                    }

                    Text {
                        anchors.verticalCenter: parent.verticalCenter
                        x: 10
                        visible: searchInput.text.length === 0
                        text: "Search timestamp or message…"
                        color: "#60778b"
                        font.pixelSize: 12
                    }
                }
            }

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 1
                color: "#26384a"
            }

            ListView {
                id: logList
                Layout.fillWidth: true
                Layout.fillHeight: true
                clip: true
                model: root.filteredEntries()
                spacing: 4
                boundsBehavior: Flickable.StopAtBounds
                onCountChanged: positionViewAtEnd()

                delegate: Rectangle {
                    id: logEntry
                    required property var modelData
                    required property int index

                    width: logList.width
                    implicitHeight: Math.max(48, messageColumn.implicitHeight + 16)
                    radius: 5
                    color: root.categoryBackground(logEntry.modelData.category, logEntry.index % 2 === 1)

                    Rectangle {
                        width: 4
                        height: parent.height
                        radius: 2
                        color: root.categoryColor(logEntry.modelData.category)
                    }

                    Rectangle {
                        x: 12
                        anchors.verticalCenter: parent.verticalCenter
                        width: 92
                        height: 22
                        radius: 4
                        color: "#0b1119"
                        border.color: root.categoryColor(logEntry.modelData.category)

                        Text {
                            anchors.centerIn: parent
                            text: root.categoryTitle(logEntry.modelData.category)
                            color: root.categoryColor(logEntry.modelData.category)
                            font.pixelSize: 9
                            font.bold: true
                        }
                    }

                    Text {
                        id: timestampText
                        x: 116
                        anchors.verticalCenter: parent.verticalCenter
                        width: 108
                        text: logEntry.modelData.timestamp || ""
                        color: "#8ca2b5"
                        font.family: "monospace"
                        font.pixelSize: 10
                        elide: Text.ElideRight
                    }

                    Column {
                        id: messageColumn
                        x: 234
                        y: 8
                        width: parent.width - x - 12
                        spacing: 3

                        readonly property bool hasInterpretation: String(logEntry.modelData.interpretation || "").length > 0

                        Text {
                            width: parent.width
                            text: messageColumn.hasInterpretation ? logEntry.modelData.interpretation : logEntry.modelData.message
                            color: "#e7eef4"
                            font.pixelSize: 12
                            font.bold: messageColumn.hasInterpretation
                            wrapMode: Text.Wrap
                        }

                        Text {
                            width: parent.width
                            visible: messageColumn.hasInterpretation
                            text: "RAW  " + (logEntry.modelData.message || "")
                            color: "#7890a3"
                            font.family: "monospace"
                            font.pixelSize: 9
                            wrapMode: Text.WrapAnywhere
                        }
                    }
                }

                Text {
                    anchors.centerIn: parent
                    visible: logList.count === 0
                    text: root.allEntries.length === 0 ? "Waiting for log activity…" : "No entries match the current filters"
                    color: "#60778b"
                    font.pixelSize: 12
                }
            }
        }
    }
}
