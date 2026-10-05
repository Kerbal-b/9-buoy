import QtQuick
import QtQuick.Controls
import QtQuick.Dialogs
import QtQuick.Layouts

Item {
    id: root
    property var backendObject
    property string pendingRemotePath: ""
    property string pendingSessionPath: ""
    property string deleteSessionPath: ""
    property string selectedSessionPath: ""

    function formatBytes(value) {
        const bytes = Number(value) || 0
        if (bytes < 1024) return bytes + " B"
        if (bytes < 1048576) return (bytes / 1024).toFixed(1) + " KB"
        if (bytes < 1073741824) return (bytes / 1048576).toFixed(2) + " MB"
        return (bytes / 1073741824).toFixed(2) + " GB"
    }

    function cardUsedPercent() {
        const card = backendObject ? backendObject.state.sdCardCapacity : ({})
        const total = Number(card.totalBytes) || 0
        return total > 0 ? Math.min(100, 100 * (Number(card.usedBytes) || 0) / total) : 0
    }

    function sessionTitle(name) {
        const match = /^session(\d+)/i.exec(String(name))
        return match ? "Session " + match[1] : String(name)
    }

    function sessionDate(name) {
        const match = /(?:^session\d+-|^session-)(\d{4})(\d{2})(\d{2})T(\d{2})(\d{2})(\d{2})Z/i.exec(String(name))
        return match ? match[1] + "-" + match[2] + "-" + match[3] + "  " + match[4] + ":" + match[5] + ":" + match[6] + " UTC" : "Date unavailable for this older session"
    }

    function formatDuration(seconds) {
        if (Number(seconds) < 0 || !isFinite(Number(seconds))) return "Estimating..."
        const value = Math.max(0, Math.round(Number(seconds)))
        const hours = Math.floor(value / 3600)
        const minutes = Math.floor((value % 3600) / 60)
        const secs = value % 60
        return (hours > 0 ? hours + "h " : "") + minutes + "m " + secs + "s"
    }

    FileDialog {
        id: saveDialog
        title: "Save SD card file locally"
        fileMode: FileDialog.SaveFile
        currentFile: backendObject ? backendObject.sdDownloadSuggestedUrl : ""
        onAccepted: {
            if (backendObject)
                backendObject.downloadSdFileFromUrl(root.pendingRemotePath, selectedFile)
        }
    }

    FolderDialog {
        id: sessionFolderDialog
        title: "Choose where to download this session"
        currentFolder: backendObject ? backendObject.sdDownloadFolderUrl : ""
        onAccepted: if (backendObject) backendObject.downloadSdSessionFromUrl(root.pendingSessionPath, selectedFolder)
    }

    Dialog {
        id: deleteSessionDialog
        title: "Permanently delete this session?"
        modal: true
        anchors.centerIn: parent
        width: 430
        padding: 16
        contentItem: Text {
            text: "This removes all files in " + root.deleteSessionPath.split("/").pop() + " from the buoy SD card. This cannot be undone."
            color: "#e6ecf2"
            wrapMode: Text.WordWrap
            width: 360
        }
        footer: RowLayout {
            spacing: 8
            Button {
                text: "Cancel"
                onClicked: deleteSessionDialog.close()
            }
            Button {
                text: "Delete session"
                palette.buttonText: "#ffffff"
                background: Rectangle { radius: 5; color: "#9e3939" }
                onClicked: {
                    root.selectedSessionPath = ""
                    if (backendObject) backendObject.deleteSdSession(root.deleteSessionPath)
                    deleteSessionDialog.close()
                }
            }
        }
    }

    Rectangle {
        anchors.fill: parent
        radius: 12
        color: "#0b1119"
        border.color: "#395166"

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 20
            spacing: 12

            RowLayout {
                Layout.fillWidth: true
                Text {
                    text: "SD Card"
                    color: "#e6ecf2"
                    font.pixelSize: 23
                    font.bold: true
                    Layout.fillWidth: true
                }
                Button {
                    text: backendObject && backendObject.sdBusy ? "Working..." : "Refresh card"
                    enabled: backendObject && !backendObject.sdBusy
                    onClicked: if (backendObject) backendObject.refreshSdFiles()
                    palette.buttonText: "#f0f5f9"
                    background: Rectangle { radius: 6; color: parent.enabled ? "#1b3448" : "#17232e"; border.color: "#55748d" }
                }
            }

            RowLayout {
                Layout.fillWidth: true
                Text {
                    text: backendObject ? backendObject.sdCardStatus : "Not connected"
                    color: "#9eb5c7"
                    Layout.fillWidth: true
                }
                Text {
                    text: backendObject ? backendObject.sdTransferStatus : "Idle"
                    color: "#72d6a4"
                    horizontalAlignment: Text.AlignRight
                    Layout.fillWidth: true
                    elide: Text.ElideMiddle
                }
            }

            ProgressBar {
                Layout.fillWidth: true
                visible: backendObject && backendObject.sdBusy
                from: 0
                to: 100
                value: backendObject ? backendObject.sdTransferProgress : 0
            }

            Rectangle {
                Layout.fillWidth: true
                visible: backendObject && (backendObject.state.sdDownloadActive || backendObject.state.serviceMode)
                implicitHeight: serviceRow.implicitHeight + 16
                radius: 7
                color: "#17352f"
                border.color: "#4caa85"
                RowLayout {
                    id: serviceRow
                    anchors.fill: parent
                    anchors.margins: 8
                    spacing: 10
                    Text {
                        Layout.fillWidth: true
                        text: backendObject
                            ? "SERVICE MODE  |  Sensors standby  |  " + (backendObject.state.sdDownloadActive
                                ? "Downloading " + backendObject.state.sdDownloadSessionPath
                                : (String(backendObject.sdTransferStatus).startsWith("Deleting ") ? backendObject.sdTransferStatus : "Finishing transfer"))
                            : ""
                        color: "#b6f0d0"
                        font.bold: true
                        elide: Text.ElideMiddle
                    }
                    Button {
                        text: "Stop download"
                        visible: backendObject && backendObject.state.sdDownloadActive
                        onClicked: if (backendObject) backendObject.cancelSdDownload()
                        background: Rectangle { radius: 5; color: "#73413c"; border.color: "#c17b72" }
                        palette.buttonText: "#ffffff"
                    }
                }
            }

            RowLayout {
                Layout.fillWidth: true
                visible: backendObject && backendObject.state.sdTransferTotalBytes > 0
                Text {
                    text: backendObject ? root.formatBytes(backendObject.state.sdTransferBytes) + " / " + root.formatBytes(backendObject.state.sdTransferTotalBytes) : ""
                    color: "#c9d8e4"
                    Layout.fillWidth: true
                }
                Text { text: backendObject ? root.formatBytes(backendObject.state.sdTransferSpeedBytesPerSecond) + "/s" : ""; color: "#75d9a7" }
                Text { text: backendObject ? "Elapsed " + root.formatDuration(backendObject.state.sdTransferElapsedSeconds) : ""; color: "#9eb5c7" }
                Text { text: backendObject ? "Left " + root.formatDuration(backendObject.state.sdTransferEtaSeconds) : ""; color: "#9eb5c7" }
            }

            Rectangle {
                Layout.fillWidth: true
                radius: 8
                color: "#101a25"
                border.color: "#30465a"
                implicitHeight: 66
                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 14
                    ColumnLayout {
                        Layout.fillWidth: true
                        Text {
                            property var card: backendObject ? backendObject.state.sdCardCapacity : ({})
                            text: "Usable " + root.formatBytes(card.totalBytes || 0) + "   |   Used " + root.formatBytes(card.usedBytes || 0) + "   |   Free " + root.formatBytes(card.freeBytes || 0)
                            color: "#e6ecf2"
                            font.bold: true
                        }
                        Rectangle {
                            Layout.fillWidth: true
                            height: 9
                            radius: 4
                            color: "#243545"
                            Rectangle { width: parent.width * root.cardUsedPercent() / 100; height: parent.height; radius: 4; color: "#43b985" }
                        }
                    }
                    Text {
                        property var card: backendObject ? backendObject.state.sdCardCapacity : ({})
                        text: "Card " + root.formatBytes(card.cardBytes || 0) + "\n" + root.cardUsedPercent().toFixed(1) + "% used"
                        color: "#9eb5c7"
                        horizontalAlignment: Text.AlignRight
                    }
                }
            }

            Rectangle {
                Layout.fillWidth: true
                Layout.fillHeight: true
                color: "#101a25"
                radius: 9
                border.color: "#30465a"

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 4

                    RowLayout {
                        Layout.fillWidth: true
                        height: 34
                        Text { text: "Buoy files by session"; color: "#e6ecf2"; Layout.fillWidth: true; font.bold: true }
                    }

                    Rectangle { Layout.fillWidth: true; height: 1; color: "#30465a" }

                    ScrollView {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        clip: true

                        Column {
                            width: parent.width
                            spacing: 3

                            Repeater {
                                model: backendObject ? backendObject.state.sdSessions : []
                                delegate: Rectangle {
                                    id: sessionCard
                                    required property var modelData
                                    width: parent.width
                                    implicitHeight: sessionColumn.implicitHeight + 16
                                    radius: 8
                                    color: "#121f2d"
                                    border.color: "#30465a"
                                    Column {
                                        id: sessionColumn
                                        anchors.left: parent.left
                                        anchors.right: parent.right
                                        anchors.top: parent.top
                                        anchors.margins: 8
                                        spacing: 5
                                        RowLayout {
                                            width: sessionColumn.width
                                            height: 56
                                            spacing: 8
                                            Rectangle {
                                                Layout.fillWidth: true
                                                Layout.fillHeight: true
                                                radius: 5
                                                color: "#1b3448"
                                                border.color: "#45627b"
                                                RowLayout {
                                                    anchors.fill: parent
                                                    anchors.leftMargin: 10
                                                    anchors.rightMargin: 10
                                                    spacing: 10
                                                    Text { text: root.selectedSessionPath === modelData.path ? "▾" : "▸"; color: "#65bdff"; font.pixelSize: 18 }
                                                    ColumnLayout {
                                                        Layout.fillWidth: true
                                                        spacing: 1
                                                        Text { text: root.sessionTitle(modelData.name); color: "#e6ecf2"; font.bold: true; font.pixelSize: 14 }
                                                        Text { text: root.sessionDate(modelData.name); color: "#9eb5c7"; font.pixelSize: 11 }
                                                    }
                                                    Text {
                                                        text: backendObject && backendObject.state.scienceSessionPath === modelData.path && backendObject.state.scienceExperimentState !== "IDLE" ? "Size after save" : root.formatBytes(modelData.size)
                                                        color: "#75d9a7"
                                                        font.bold: true
                                                    }
                                                }
                                                MouseArea {
                                                    anchors.fill: parent
                                                    onClicked: {
                                                        root.selectedSessionPath = root.selectedSessionPath === modelData.path ? "" : modelData.path
                                                        if (root.selectedSessionPath && !modelData.loaded && backendObject)
                                                            backendObject.loadSdSession(modelData.path)
                                                    }
                                                }
                                            }
                                            Button {
                                                text: "Download session"
                                                enabled: backendObject && !backendObject.sdBusy && backendObject.state.scienceExperimentState === "IDLE"
                                                onClicked: { root.pendingSessionPath = modelData.path; sessionFolderDialog.open() }
                                                palette.buttonText: "#f0f5f9"
                                                background: Rectangle { radius: 5; color: parent.enabled ? "#1b3448" : "#17232e"; border.color: "#55748d" }
                                            }
                                            Button {
                                                text: "Delete"
                                                enabled: backendObject && !backendObject.sdBusy && backendObject.state.scienceExperimentState === "IDLE"
                                                onClicked: {
                                                    root.deleteSessionPath = modelData.path
                                                    deleteSessionDialog.open()
                                                }
                                                palette.buttonText: "#ffffff"
                                                background: Rectangle { radius: 5; color: parent.enabled ? "#713b3b" : "#17232e"; border.color: "#9e5555" }
                                            }
                                        }
                                        Text { visible: root.selectedSessionPath === modelData.path && !modelData.loaded; text: "Loading files when opened..."; color: "#9eb5c7" }
                                        Text { visible: root.selectedSessionPath === modelData.path && modelData.loaded && modelData.files.length === 0; text: "No science files in this session"; color: "#9eb5c7" }
                                        Repeater {
                                            model: modelData.files
                                            delegate: Rectangle {
                                                required property var modelData
                                                width: sessionColumn.width
                                                visible: root.selectedSessionPath === sessionCard.modelData.path
                                                height: visible ? 48 : 0
                                                radius: 5
                                                color: "#0d1721"
                                                RowLayout {
                                                    anchors.fill: parent
                                                    anchors.leftMargin: 9
                                                    anchors.rightMargin: 7
                                                    spacing: 8
                                                    ColumnLayout {
                                                        Layout.fillWidth: true
                                                        spacing: 1
                                                        Text { text: modelData.category + "  |  " + modelData.name; color: modelData.category === "SCIENCE" ? "#75d9a7" : (modelData.category === "AUDIO" ? "#e8b968" : "#e6ecf2"); font.bold: true; elide: Text.ElideRight; Layout.fillWidth: true }
                                                        Text { text: modelData.description + "  -  " + root.formatBytes(modelData.size); color: "#9eb5c7"; font.pixelSize: 11; elide: Text.ElideRight; Layout.fillWidth: true }
                                                    }
                                                    Button {
                                                        text: "Download"
                                                        enabled: backendObject && !backendObject.sdBusy && backendObject.state.scienceExperimentState === "IDLE"
                                                        onClicked: {
                                                            root.pendingRemotePath = modelData.path
                                                            backendObject.prepareSdDownload(modelData.path)
                                                            saveDialog.open()
                                                        }
                                                        palette.buttonText: "#f0f5f9"
                                                        background: Rectangle { radius: 5; color: parent.enabled ? "#1b3448" : "#17232e"; border.color: "#55748d" }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }

                    Text {
                        visible: backendObject && !backendObject.sdBusy && backendObject.state.sdSessions.length === 0
                        text: "No buoy session files found. Connect over Wi-Fi and refresh the card."
                        color: "#829aad"
                        Layout.alignment: Qt.AlignHCenter
                        Layout.topMargin: 12
                    }
                }
            }

            Text {
                text: "Card usage includes all SD files; session folders show total file size. Open a folder to load its files. Downloads and session deletion require a saved mission and put the buoy in service mode with sensors on standby. Deletion permanently erases that session from the SD card."
                color: "#829aad"
                font.pixelSize: 11
                wrapMode: Text.WordWrap
                Layout.fillWidth: true
            }
        }
    }
}
