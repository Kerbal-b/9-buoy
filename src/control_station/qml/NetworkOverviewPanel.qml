import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Item {
    id: root
    property var backendObject
    property var backendState: ({})

    readonly property bool onBuoyNetwork: /^192\.168\.8\.\d+$/.test(String(backendObject ? backendObject.stationNetworkIp : ""))
    readonly property bool buoyWifiConnected: String(backendState.serialStatus || "").startsWith("Connected")
        && String(backendState.serialTarget || "").startsWith("WIFI:")
    readonly property bool backpackFound: Boolean(backendState.radioWifiIp)
    readonly property bool backpackTelemetry: Boolean(backendState.radioBuoyState)
    readonly property bool rfObserved: /\d+%/.test(String(backendState.radioLinkQuality || ""))
        && !String(backendState.radioLinkStatus || "").toLowerCase().includes("stale")
        && !String(backendState.radioLinkStatus || "").toLowerCase().includes("via buoy wi-fi")

    function buoyAddressSummary() {
        const target = String(backendState.serialTarget || "")
        if (target.startsWith("WIFI:")) {
            const address = target.substring(5).split(":")[0]
            if (address && address.toLowerCase() !== "auto")
                return (root.buoyWifiConnected ? "Connected at " : "Target ") + address
        }
        const devices = backendState.networkBuoyDevices || []
        const addresses = devices.map(device => String(device.ip || "")).filter(Boolean)
        if (addresses.length === 1) return "Reachable at " + addresses[0]
        if (addresses.length > 1) return "Reachable at " + addresses.join(", ")
        return "Buoy IP not found"
    }

    function interfaceIndex() {
        const interfaces = backendObject ? backendObject.networkInterfaces : []
        for (let i = 0; i < interfaces.length; ++i)
            if (interfaces[i].value === backendObject.wifiLocalIp) return i
        return 0
    }

    component PanelButton: Button {
        id: button
        property bool primary: false
        implicitHeight: 36
        implicitWidth: Math.max(90, label.implicitWidth + 22)
        background: Rectangle {
            radius: 8
            color: !button.enabled ? "#172431" : button.primary ? "#175662" : button.hovered ? "#263c4d" : "#1c2d3b"
            border.color: !button.enabled ? "#263643" : button.primary ? "#3c9ca7" : "#385064"
        }
        contentItem: Text {
            id: label
            text: button.text
            color: button.enabled ? "#e6ecf2" : "#657b8b"
            font.pixelSize: 11
            font.bold: true
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
        }
    }

    ScrollView {
        anchors.fill: parent
        clip: true
        contentWidth: availableWidth
        ScrollBar.vertical.policy: ScrollBar.AsNeeded

        ColumnLayout {
            width: Math.max(0, parent.width - 28)
            x: 14
            y: 14
            spacing: 12

            RowLayout {
                Layout.fillWidth: true
                spacing: 10
                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 2
                    Text { text: "Network connections"; color: "#e6ecf2"; font.pixelSize: 22; font.bold: true }
                    Text {
                        text: String(backendState.telemetryMode || "WIFI").toUpperCase() === "MAVLINK"
                            ? "SF / CH6 HIGH routes reduced-rate MAVLink telemetry through the Backpack. Buoy Wi-Fi remains a separate command and safety-status path."
                            : "SF / CH6 LOW or MIDDLE routes detailed telemetry over buoy Wi-Fi; Backpack MAVLink carries only a heartbeat."
                        color: "#91a8b9"; font.pixelSize: 11; wrapMode: Text.WordWrap; Layout.fillWidth: true
                    }
                }
                PanelButton { text: backendObject ? backendObject.connectionButtonLabel : "Connect"; primary: true; enabled: Boolean(backendObject) && text !== "Connecting..." && text !== "Reconnecting..."; onClicked: backendObject.toggleConnection() }
            }

            NetworkDiagram {
                Layout.fillWidth: true
                Layout.preferredHeight: 285
                backendObject: root.backendObject
                backendState: root.backendState
                onBuoyNetwork: root.onBuoyNetwork
                buoyWifiConnected: root.buoyWifiConnected
                backpackFound: root.backpackFound
                backpackTelemetry: root.backpackTelemetry
                rfObserved: root.rfObserved
            }

            GridLayout {
                Layout.fillWidth: true
                columns: width >= 900 ? 2 : 1
                columnSpacing: 12
                rowSpacing: 12

                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: connectionSetup.implicitHeight + 28
                    radius: 11; color: "#121c28"; border.color: "#3b5064"
                    RowLayout {
                        anchors.fill: parent
                        spacing: 0
                        Rectangle {
                            Layout.preferredWidth: 8
                            Layout.fillHeight: true
                            radius: 4
                            color: root.buoyWifiConnected ? "#58d7a5" : "#607789"
                        }
                        ColumnLayout {
                            id: connectionSetup
                            Layout.fillWidth: true
                            Layout.margins: 13
                            spacing: 9
                            RowLayout {
                                Layout.fillWidth: true
                                Text { text: "Buoy Wi-Fi connection"; color: "#e6ecf2"; font.pixelSize: 14; font.bold: true; Layout.fillWidth: true }
                                Text { text: root.buoyWifiConnected ? "CONNECTED" : "NOT CONNECTED"; color: root.buoyWifiConnected ? "#58d7a5" : "#91a8b9"; font.pixelSize: 9; font.bold: true }
                            }
                            RowLayout {
                                Layout.fillWidth: true
                                spacing: 9
                                Text { text: "Network adapter"; color: "#91a8b9"; font.pixelSize: 11 }
                                ComboBox {
                                    id: adapterBox
                                    Layout.fillWidth: true
                                    Layout.minimumWidth: 160
                                    implicitHeight: 38
                                    model: backendObject ? backendObject.networkInterfaces : []
                                    textRole: "label"
                                    valueRole: "value"
                                    currentIndex: root.interfaceIndex()
                                    onActivated: if (backendObject) backendObject.setWifiLocalIp(currentValue)
                                    background: Rectangle { radius: 7; color: "#101b27"; border.color: adapterBox.activeFocus ? "#55aab8" : "#385064" }
                                    contentItem: Text {
                                        leftPadding: 10
                                        rightPadding: 28
                                        text: adapterBox.displayText
                                        color: "#e6ecf2"
                                        font.pixelSize: 11
                                        verticalAlignment: Text.AlignVCenter
                                        elide: Text.ElideMiddle
                                    }
                                    indicator: Text { x: adapterBox.width - width - 10; y: (adapterBox.height - height) / 2; text: "v"; color: "#91a8b9"; font.pixelSize: 12 }
                                    delegate: ItemDelegate {
                                        width: adapterBox.width
                                        height: 44
                                        highlighted: adapterBox.highlightedIndex === index
                                        contentItem: Column {
                                            anchors.verticalCenter: parent.verticalCenter
                                            spacing: 2
                                            Text { text: modelData.label; color: "#e6ecf2"; font.pixelSize: 11; elide: Text.ElideMiddle; width: parent.width }
                                            Text { text: modelData.value === "auto" ? "Search all detected adapters" : "IPv4  " + modelData.value; color: "#91a8b9"; font.pixelSize: 9; elide: Text.ElideMiddle; width: parent.width }
                                        }
                                        background: Rectangle { color: parent.highlighted ? "#263c4d" : "#172431" }
                                    }
                                    popup: Popup {
                                        y: adapterBox.height + 2
                                        width: adapterBox.width
                                        implicitHeight: Math.min(contentItem.implicitHeight + 8, 260)
                                        padding: 4
                                        contentItem: ListView {
                                            clip: true
                                            implicitHeight: contentHeight
                                            model: adapterBox.delegateModel
                                            currentIndex: adapterBox.highlightedIndex
                                            ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }
                                        }
                                        background: Rectangle { radius: 8; color: "#172431"; border.color: "#385064" }
                                    }
                                }
                                PanelButton { text: backendState.networkScanning ? "Scanning..." : "Scan buoy"; primary: true; enabled: Boolean(backendObject) && !Boolean(backendState.networkScanning); onClicked: backendObject.scanBuoyNetwork() }
                            }
                            Text { text: root.buoyAddressSummary(); color: root.buoyWifiConnected || (backendState.networkBuoyDevices || []).length ? "#dce8ef" : "#91a8b9"; font.pixelSize: 11; font.bold: true; wrapMode: Text.WordWrap; Layout.fillWidth: true }
                            Text { text: backendState.networkScanStatus || "Choose the adapter connected to 192.168.8.0/24, then scan for buoy devices."; color: "#91a8b9"; font.pixelSize: 10; wrapMode: Text.WordWrap; Layout.fillWidth: true }
                        }
                    }
                }

                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: backpackSetup.implicitHeight + 28
                    radius: 11; color: "#121c28"; border.color: "#3b5064"
                    RowLayout {
                        anchors.fill: parent
                        spacing: 0
                        Rectangle {
                            Layout.preferredWidth: 8
                            Layout.fillHeight: true
                            radius: 4
                            color: root.backpackTelemetry ? "#58d7a5" : root.backpackFound ? "#f1bd68" : "#607789"
                        }
                        ColumnLayout {
                            id: backpackSetup
                            Layout.fillWidth: true
                            Layout.margins: 13
                            spacing: 8
                            RowLayout {
                                Layout.fillWidth: true
                                Text { text: "TX Backpack"; color: "#e6ecf2"; font.pixelSize: 14; font.bold: true; Layout.fillWidth: true }
                                PanelButton { text: backendState.radioWifiScanning ? "Searching..." : "Find backpack"; primary: true; enabled: Boolean(root.backendObject) && !Boolean(backendState.radioWifiScanning); onClicked: root.backendObject.findRadioBackpack() }
                            }
                            Text {
                                Layout.fillWidth: true
                                text: root.backpackFound ? String(backendState.radioWifiIp) + "  ·  " + String(backendState.radioWifiStatus || "Backpack reachable") : String(backendState.radioWifiStatus || "Backpack not found")
                                color: root.backpackFound ? "#dce8ef" : "#91a8b9"
                                font.pixelSize: 11
                                wrapMode: Text.WordWrap
                            }
                            Text {
                                Layout.fillWidth: true
                                text: "Station packets: " + String(backendState.radioTelemetryPackets || 0) + "  ·  " + String(backendState.radioTelemetryStatus || "Waiting for telemetry")
                                color: root.backpackTelemetry ? "#58d7a5" : "#f1bd68"
                                font.pixelSize: 10
                                wrapMode: Text.WordWrap
                            }
                        }
                    }
                }
            }

            Rectangle {
                Layout.fillWidth: true
                implicitHeight: telemetryModes.implicitHeight + 26
                radius: 11; color: "#101b27"; border.color: "#30485c"
                ColumnLayout {
                    id: telemetryModes
                    anchors.fill: parent; anchors.margins: 13; spacing: 6
                    Text { text: "WHAT TO EXPECT FROM TELEMETRY"; color: "#7abfff"; font.pixelSize: 11; font.bold: true }
                    Text {
                        Layout.fillWidth: true
                        text: "CH6 LOW / MIDDLE · Buoy Wi-Fi: detailed sensor, GPS, control, and receiver-link telemetry. ELRS MAVLink is heartbeat-only."
                        color: "#dce8ef"; font.pixelSize: 10; wrapMode: Text.WordWrap
                    }
                    Text {
                        Layout.fillWidth: true
                        text: "CH6 HIGH · Backpack MAVLink: buoy state, arm/session/audio state, receiver link/channels, battery, GPS, and rotating science values. Buoy Wi-Fi still carries commands and compact safety status; high-rate IMU/audio and SD transfers need Wi-Fi."
                        color: "#dce8ef"; font.pixelSize: 10; wrapMode: Text.WordWrap
                    }
                    Text {
                        Layout.fillWidth: true
                        text: root.backpackTelemetry ? "Backpack telemetry is arriving." : (root.onBuoyNetwork
                            ? (String(backendState.telemetryMode || "WIFI").toUpperCase() === "WIFI"
                                ? "Backpack is unavailable, but CH6 selects Wi-Fi and this station is on the buoy network, so detailed Wi-Fi telemetry remains available."
                                : "Backpack packets are missing. On buoy Wi-Fi, compact safety status may still appear, but Backpack-delivered state and low-rate measurements will be unavailable.")
                            : (String(backendState.telemetryMode || "WIFI").toUpperCase() === "MAVLINK"
                                ? "No Backpack packets and no buoy Wi-Fi connection: no buoy telemetry is available to this station. TX12 control may still work locally."
                                : "CH6 selects Wi-Fi, but this station is not connected to buoy Wi-Fi: no detailed buoy telemetry is available."))
                        color: root.backpackTelemetry ? "#58d7a5" : "#f1bd68"; font.pixelSize: 10; wrapMode: Text.WordWrap
                    }
                }
            }
        }
    }
}
