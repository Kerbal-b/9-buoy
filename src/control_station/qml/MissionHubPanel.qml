import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: root
    property var backendObject
    property var backendState: ({})
    property int sectionIndex: 1
    onVisibleChanged: if (visible) sectionIndex = 1

    ColumnLayout {
        anchors.fill: parent
        spacing: 8
        TabBar {
            id: missionTabs
            Layout.fillWidth: true
            currentIndex: root.sectionIndex
            onCurrentIndexChanged: root.sectionIndex = currentIndex
            TabButton { text: "Mission Creation"; implicitHeight: 38; font.pixelSize: 13 }
            TabButton { text: "Mission Control"; implicitHeight: 38; font.pixelSize: 13 }
        }
        StackLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: root.sectionIndex
            MissionsPanel {
                Layout.fillWidth: true
                Layout.fillHeight: true
                backendObject: root.backendObject
                backendState: root.backendState
            }
            MissionProgressPanel {
                Layout.fillWidth: true
                Layout.fillHeight: true
                backendObject: root.backendObject
                backendState: root.backendState
            }
        }
    }
}
