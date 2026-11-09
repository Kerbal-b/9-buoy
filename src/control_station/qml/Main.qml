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
    function keyName(key) {
        if (key === Qt.Key_Left) return "left"
        if (key === Qt.Key_Right) return "right"
        if (key === Qt.Key_Up) return "up"
        if (key === Qt.Key_Down) return "down"
        return ""
    }

    function dashboardTabIndex(tab) {
        if (tab === "communications") return 1
        if (tab === "power") return 2
        if (tab === "motors") return 3
        if (tab === "navigation") return 4
        if (tab === "missions") return 5
        if (tab === "instrument_debug") return 6
        if (tab === "sd_debug") return 7
        return 0
    }

    function dashboardTabForIndex(index) {
        if (index === 1) return "communications"
        if (index === 2) return "power"
        if (index === 3) return "motors"
        if (index === 4) return "navigation"
        if (index === 5) return "missions"
        if (index === 6) return "instrument_debug"
        if (index === 7) return "sd_debug"
        return "overview"
    }

    function handleKey(event, pressed) {
        if (!Boolean(s.keyboardDriveEnabled)) {
            return false
        }
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

    BuoySidebar {
        id: buoyPanel
        x: 20
        y: 20
        width: 280
        height: parent.height - 40
        backendObject: backend
        backendState: s
    }

    Rectangle {
        id: dashboardPanel
        x: buoyPanel.x + buoyPanel.width + 14
        y: 20
        width: parent.width - x - 20
        height: parent.height - 40
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
                    propulsionPanel.activate()
                } else {
                    propulsionPanel.deactivate()
                }
                if (tab === "sd_debug") backend.refreshSdFiles()
            }

            TabButton { text: "Overview"; font.pixelSize: 12 }
            TabButton { text: "Communications"; font.pixelSize: 12 }
            TabButton { text: "Power"; font.pixelSize: 12 }
            TabButton { text: "Propulsion"; font.pixelSize: 12 }
            TabButton { text: "Navigation"; font.pixelSize: 12 }
            TabButton { text: "Mission"; font.pixelSize: 12 }
            TabButton { text: "Instrument Debugging"; font.pixelSize: 12 }
            TabButton { text: "SD Card"; font.pixelSize: 12 }
        }

        StackLayout {
            x: 16
            y: 96
            width: parent.width - 32
            height: parent.height - 112
            currentIndex: dashboardTabIndex(backend.dashboardTab)

            OverviewPanel {
                Layout.fillWidth: true
                Layout.fillHeight: true
                backendObject: backend
                backendState: s
            }

            CommunicationsPanel {
                Layout.fillWidth: true
                Layout.fillHeight: true
                backendObject: backend
                backendState: s
            }

            PowerPanel {
                Layout.fillWidth: true
                Layout.fillHeight: true
                backendObject: backend
                backendState: s
            }

            PropulsionPanel {
                id: propulsionPanel
                Layout.fillWidth: true
                Layout.fillHeight: true
                backendObject: backend
                backendState: s
            }

            NavigationDashboardPanel {
                Layout.fillWidth: true
                Layout.fillHeight: true
                backendObject: backend
                backendState: s
            }

            MissionHubPanel {
                Layout.fillWidth: true
                Layout.fillHeight: true
                backendObject: backend
                backendState: s
            }

            InstrumentPanel {
                Layout.fillWidth: true
                Layout.fillHeight: true
                backendObject: backend
                backendState: s
            }

            SdCardDebugPanel {
                Layout.fillWidth: true
                Layout.fillHeight: true
                backendObject: backend
            }
        }
    }
}


