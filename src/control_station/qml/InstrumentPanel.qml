import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: root
    property var backendObject
    property var backendState: ({})

    ColumnLayout {
        anchors.fill: parent
        spacing: 8
        TabBar {
            id: instrumentTabs
            Layout.fillWidth: true
            TabButton { text: "Readings & audio" }
            TabButton { text: "Microphone debugging" }
        }
        StackLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: instrumentTabs.currentIndex
            ScienceAudioPanel { backendObject: root.backendObject; backendState: root.backendState }
            MicrophoneDebugPanel { backendObject: root.backendObject; backendState: root.backendState }
        }
    }
}
