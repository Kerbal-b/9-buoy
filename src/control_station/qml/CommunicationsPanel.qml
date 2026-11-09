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
            id: sections
            Layout.fillWidth: true
            TabButton { text: "Network" }
            TabButton { text: "Radio / ELRS" }
            TabButton { text: "Link log" }
        }

        StackLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: sections.currentIndex

            NetworkOverviewPanel {
                backendObject: root.backendObject
                backendState: root.backendState
            }
            RadioElrsPanel {
                backendObject: root.backendObject
                backendState: root.backendState
            }
            LogsPanel {
                backendObject: root.backendObject
                backendState: root.backendState
                active: sections.currentIndex === 2 && root.backendObject && root.backendObject.dashboardTab === "communications"
            }
        }
    }
}
