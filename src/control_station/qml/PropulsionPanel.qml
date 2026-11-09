import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Item {
    id: root
    property var backendObject
    property var backendState: ({})

    function activate() { motorPage.activate() }
    function deactivate() { motorPage.deactivate() }

    ColumnLayout {
        anchors.fill: parent
        spacing: 8
        TabBar {
            id: propulsionTabs
            Layout.fillWidth: true
            onCurrentIndexChanged: if (currentIndex !== 0) motorPage.deactivate()
            TabButton { text: "Motor tests" }
            TabButton { text: "Mixer calibration" }
        }
        StackLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: propulsionTabs.currentIndex
            MotorTestPanel {
                id: motorPage
                Layout.fillWidth: true
                Layout.fillHeight: true
                backendObject: root.backendObject
                backendState: root.backendState
            }
            MixerCalibrationPanel {
                Layout.fillWidth: true
                Layout.fillHeight: true
                backendObject: root.backendObject
                backendState: root.backendState
            }
        }
    }
}
