import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: root
    property var backendObject
    property var backendState: ({})
    property bool showUnmappedControls: false

    function interfaceIndex() {
        const interfaces = backendObject ? backendObject.networkInterfaces : []
        for (let i = 0; i < interfaces.length; ++i) {
            if (interfaces[i].value === backendObject.wifiLocalIp) return i
        }
        return 0
    }

    function hasMappedFunction(control) {
        if (control.channel === null || control.channel === undefined) return false
        if (control.purpose && control.purpose.trim() && control.purpose !== "Unassigned") return true
        return Boolean(control.positions && control.positions.some(position => position.meaning && position.meaning.trim()))
    }

    function displayedControls() {
        const controls = backendState.tx12Controls || []
        return showUnmappedControls ? controls : controls.filter(control => hasMappedFunction(control))
    }

    function markerLabel(control) {
        const labels = {
            left_stick_horizontal: "L-H", left_stick_vertical: "L-V",
            right_stick_horizontal: "R-H", right_stick_vertical: "R-V"
        }
        return labels[control.id] || control.id.toUpperCase().slice(0, 3)
    }

    function linkQualityPercent() {
        const match = String(backendState.radioLinkQuality || "").match(/(\d+(?:\.\d+)?)%/)
        return match ? Math.max(0, Math.min(100, Number(match[1]))) : -1
    }

    function linkQualityColor(percent) {
        if (percent < 0) return "#607080"
        if (percent >= 70) return "#55d6a2"
        if (percent >= 40) return "#f2bf58"
        return "#f07878"
    }

    function sendCommand() {
        const value = commandText.text.trim()
        if (value && backendObject) backendObject.sendText(value)
    }

    function wifiBuoyConnected() {
        return String(backendState.serialStatus || "").startsWith("Connected") && String(backendState.serialTarget || "").startsWith("WIFI:")
    }

    function stationOnBuoyRouter() {
        const address = String(backendObject ? backendObject.stationNetworkIp : "")
        return /^192\.168\.8\.\d+$/.test(address)
    }

    function buoyDeviceReachable() {
        return wifiBuoyConnected() || (backendState.networkBuoyDevices || []).length > 0
    }

    function buoyStatusColor() {
        if (wifiBuoyConnected()) return "#55d6a2"
        if ((backendState.networkBuoyDevices || []).length > 0) return "#f2bf58"
        return "#536575"
    }

    function buoyControlStatus() {
        if (wifiBuoyConnected()) return "CONNECTED"
        if (String(backendState.serialStatus || "").startsWith("Connecting") || String(backendState.serialStatus || "").startsWith("Reconnecting")) return "CONNECTING"
        if ((backendState.networkBuoyDevices || []).length > 0) return "REACHABLE"
        return "OFFLINE"
    }

    function buoyControlStatusColor() {
        if (wifiBuoyConnected()) return "#55d6a2"
        if (buoyControlStatus() === "REACHABLE" || buoyControlStatus() === "CONNECTING") return "#f2bf58"
        return "#f07878"
    }

    function wifiTrafficSummary() {
        if (!wifiBuoyConnected()) return "No active buoy control link"
        return (backendState.wifiReceivedMessages || 0) + " RX msg | " + Number(backendState.wifiMessageRate || 0).toFixed(1) + " msg/s | " + Number(backendState.wifiThroughputKib || 0).toFixed(1) + " KiB/s"
    }

    function elrsLinkActive() {
        return root.linkQualityPercent() >= 0 && !String(backendState.radioLinkStatus || "").toLowerCase().includes("stale")
    }

    function elrsLinkDetails() {
        if (!root.elrsLinkActive()) return "No ELRS link telemetry"
        const details = ["LQ " + Math.round(root.linkQualityPercent()) + "%"]
        const rssi = String(backendState.radioRssiStatus || "").match(/-?\d+\s*dBm/i)
        const snr = String(backendState.radioSnrStatus || "").match(/-?\d+\s*dB/i)
        if (rssi) details.push("RSSI " + rssi[0])
        if (snr) details.push("SNR " + snr[0])
        return details.join(" | ")
    }

    function wifiNetworkColor(active) {
        return active ? "#55d6a2" : "#536575"
    }

    function buoyAddressSummary() {
        const devices = backendState.networkBuoyDevices || []
        if (devices.length) return devices.map(device => device.ip).join(" · ")
        if (wifiBuoyConnected()) {
            const match = String(backendState.serialTarget || "").match(/^WIFI:([^:]+)/)
            return match ? match[1] : "Connected buoy"
        }
        return "Buoy connection not present"
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 8
        TabBar {
            id: sections
            Layout.fillWidth: true
            TabButton { text: "Wi-Fi" }
            TabButton { text: "Radio / ELRS" }
            TabButton { text: "Link log" }
        }
        StackLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: sections.currentIndex

            Rectangle {
                radius: 12; color: "#0b1119"; border.color: "#395166"
                ColumnLayout {
                    anchors.fill: parent; anchors.margins: 12; spacing: 5
                    RowLayout {
                        Layout.fillWidth: true
                        ColumnLayout {
                            Layout.fillWidth: true; spacing: 3
                            Text { text: "Buoy network"; color: "#e6ecf2"; font.pixelSize: 22; font.bold: true }
                            Text { text: "Router presence follows the station IP on 192.168.8.0/24; buoy reachability is shown separately"; color: "#829aad"; font.pixelSize: 12; Layout.fillWidth: true; wrapMode: Text.WordWrap }
                        }
                        Button { text: backendObject ? backendObject.connectionButtonLabel : "Connect"; enabled: Boolean(backendObject) && text !== "Connecting..." && text !== "Reconnecting..."; onClicked: backendObject.toggleConnection() }
                    }
                    RowLayout {
                        Layout.fillWidth: true
                        Text { text: "Control station interface"; color: "#9eb5c7"; font.pixelSize: 11 }
                        ComboBox {
                            Layout.preferredWidth: 280
                            model: backendObject ? backendObject.networkInterfaces : []
                            textRole: "label"; valueRole: "value"
                            currentIndex: root.interfaceIndex()
                            onActivated: if (backendObject) backendObject.setWifiLocalIp(currentValue)
                        }
                        Rectangle { Layout.fillWidth: true; height: 1; color: "#263949" }
                        Rectangle {
                            Layout.preferredWidth: 180; Layout.preferredHeight: 30; radius: 7
                            color: root.stationOnBuoyRouter() ? "#102c27" : "#111c28"
                            border.color: root.wifiNetworkColor(root.stationOnBuoyRouter())
                            RowLayout {
                                anchors.fill: parent; anchors.leftMargin: 9; anchors.rightMargin: 8; spacing: 7
                                Rectangle { width: 8; height: 8; radius: 4; color: root.wifiNetworkColor(root.stationOnBuoyRouter()) }
                                Text { text: root.stationOnBuoyRouter() ? "Router network detected" : "Not on 192.168.8.0/24"; color: root.wifiNetworkColor(root.stationOnBuoyRouter()); font.pixelSize: 10; font.bold: true }
                            }
                        }
                    }

                    Item {
                        id: networkDiagram
                        Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumHeight: 450
                        Canvas {
                            id: networkLinks
                            anchors.fill: parent
                            property bool stationOnline: root.stationOnBuoyRouter()
                            property bool buoyOnline: root.wifiBuoyConnected()
                            property bool backpackOnline: Boolean(backendState.radioWifiIp)
                            property bool elrsOnline: root.elrsLinkActive()
                            onStationOnlineChanged: requestPaint()
                            onBuoyOnlineChanged: requestPaint()
                            onBackpackOnlineChanged: requestPaint()
                            onElrsOnlineChanged: requestPaint()
                            onWidthChanged: requestPaint()
                            onHeightChanged: requestPaint()
                            onPaint: {
                                const ctx = getContext("2d")
                                ctx.clearRect(0, 0, width, height)
                                const cy = height * 0.245
                                function drawLink(x1, y1, x2, y2, color, active) {
                                    ctx.beginPath()
                                    ctx.moveTo(x1, y1)
                                    ctx.quadraticCurveTo((x1 + x2) / 2, (y1 + y2) / 2 - 14, x2, y2)
                                    ctx.strokeStyle = active ? color : "#405365"
                                    ctx.lineWidth = active ? 2.5 : 1.5
                                    ctx.stroke()
                                }
                                drawLink(width * 0.17 + 80, cy, width * 0.50 - 77, cy, "#55d6a2", stationOnline)
                                drawLink(width * 0.50 + 77, cy, width * 0.83 - 85, cy, "#55d6a2", buoyOnline)
                                drawLink(width * 0.50 + 42, cy + 90, width * 0.76 - 42, height * 0.77 - 95, "#55d6a2", backpackOnline)
                                drawLink(width * 0.76 + 42, height * 0.77 - 95, width * 0.83 - 42, cy + 97, "#53c9ff", elrsOnline)
                            }
                        }
                        Rectangle {
                            x: networkDiagram.width * 0.50 - width / 2; y: networkDiagram.height * 0.245 - height / 2
                            width: 154; height: 184; radius: 13; color: "#111c28"
                            border.width: 1.5; border.color: root.wifiNetworkColor(root.stationOnBuoyRouter())
                            Image { anchors.top: parent.top; anchors.topMargin: 4; anchors.horizontalCenter: parent.horizontalCenter; width: 138; height: 120; source: "../assets/network-router.png"; sourceSize.width: 384; fillMode: Image.PreserveAspectFit; smooth: true; mipmap: true }
                            Column {
                                anchors.bottom: parent.bottom; anchors.bottomMargin: 12; anchors.horizontalCenter: parent.horizontalCenter; spacing: 3
                                Text { anchors.horizontalCenter: parent.horizontalCenter; text: "BUOY ROUTER"; color: "#e6ecf2"; font.pixelSize: 12; font.bold: true }
                                Text { anchors.horizontalCenter: parent.horizontalCenter; text: root.stationOnBuoyRouter() ? "NETWORK PRESENT" : "NETWORK NOT DETECTED"; color: root.wifiNetworkColor(root.stationOnBuoyRouter()); font.pixelSize: 9; font.bold: true }
                            }
                        }
                        Rectangle {
                            x: networkDiagram.width * 0.17 - width / 2; y: networkDiagram.height * 0.245 - height / 2
                            width: 160; height: 176; radius: 13; color: "#111c28"
                            border.width: 1.5; border.color: root.wifiNetworkColor(root.stationOnBuoyRouter())
                            Image { anchors.top: parent.top; anchors.topMargin: 5; anchors.horizontalCenter: parent.horizontalCenter; width: 130; height: 116; source: "../assets/network-control-station.png"; sourceSize.width: 384; fillMode: Image.PreserveAspectFit; smooth: true; mipmap: true }
                            Column {
                                anchors.bottom: parent.bottom; anchors.bottomMargin: 12; anchors.horizontalCenter: parent.horizontalCenter; spacing: 3
                                Text { anchors.horizontalCenter: parent.horizontalCenter; text: "CONTROL STATION"; color: "#e6ecf2"; font.pixelSize: 11; font.bold: true }
                                Text { anchors.horizontalCenter: parent.horizontalCenter; text: backendObject ? backendObject.stationNetworkIp : ""; color: "#9eb5c7"; font.pixelSize: 10 }
                            }
                        }
                        Rectangle {
                            x: networkDiagram.width * 0.83 - width / 2; y: networkDiagram.height * 0.245 - height / 2
                            width: 170; height: 198; radius: 13; color: "#111c28"
                            border.width: 1.5; border.color: root.buoyStatusColor()
                            Image { anchors.top: parent.top; anchors.topMargin: 1; anchors.horizontalCenter: parent.horizontalCenter; width: 130; height: 112; source: "../assets/network-buoy.png"; sourceSize.width: 320; fillMode: Image.PreserveAspectFit; smooth: true; mipmap: true }
                            Column {
                                anchors.bottom: parent.bottom; anchors.bottomMargin: 9; anchors.horizontalCenter: parent.horizontalCenter; width: parent.width - 16; spacing: 3
                                Text { anchors.horizontalCenter: parent.horizontalCenter; text: (backendState.networkBuoyDevices || []).length > 1 ? "BUOY DEVICES" : "BUOY DEVICE"; color: "#e6ecf2"; font.pixelSize: 11; font.bold: true }
                                Text { width: parent.width; horizontalAlignment: Text.AlignHCenter; text: (root.wifiBuoyConnected() ? "CONNECTED - " : (backendState.networkBuoyDevices || []).length > 0 ? "REACHABLE - " : "OFFLINE - ") + root.buoyAddressSummary(); color: root.buoyStatusColor(); font.pixelSize: 9; elide: Text.ElideMiddle }
                                Text { anchors.horizontalCenter: parent.horizontalCenter; text: "TCP " + (backendState.tcpPort || 5000) + "  ·  UDP " + (backendState.udpPort || 5001) + "  ·  AUDIO " + (backendState.audioPort || 5002); color: "#829aad"; font.pixelSize: 8 }
                            }
                        }
                        Rectangle {
                            x: networkDiagram.width * 0.76 - width / 2; y: networkDiagram.height * 0.77 - height / 2
                            width: 170; height: 190; radius: 13; color: "#111c28"
                            border.width: 1.5; border.color: root.wifiNetworkColor(Boolean(backendState.radioWifiIp))
                            Image { anchors.top: parent.top; anchors.topMargin: 2; anchors.horizontalCenter: parent.horizontalCenter; width: 145; height: 130; source: "../assets/tx12-mkii-front-white.png"; sourceSize.width: 320; fillMode: Image.PreserveAspectFit; smooth: true; mipmap: true }
                            Column {
                                anchors.bottom: parent.bottom; anchors.bottomMargin: 10; anchors.horizontalCenter: parent.horizontalCenter; spacing: 3
                                Text { anchors.horizontalCenter: parent.horizontalCenter; text: "TX12 · ELRS BACKPACK"; color: "#e6ecf2"; font.pixelSize: 10; font.bold: true }
                                Text { anchors.horizontalCenter: parent.horizontalCenter; text: backendState.radioWifiIp || "Backpack not found"; color: root.wifiNetworkColor(Boolean(backendState.radioWifiIp)); font.pixelSize: 9 }
                            }
                        }
                        Repeater {
                            model: [
                                { x: networkDiagram.width * 0.335, y: networkDiagram.height * 0.245, active: root.stationOnBuoyRouter() },
                                { x: networkDiagram.width * 0.665, y: networkDiagram.height * 0.245, active: root.wifiBuoyConnected() },
                                { x: networkDiagram.width * 0.63, y: networkDiagram.height * 0.505, active: Boolean(backendState.radioWifiIp) },
                                { x: networkDiagram.width * 0.795, y: networkDiagram.height * 0.505, active: root.elrsLinkActive() }
                            ]
                            delegate: Rectangle {
                                required property var modelData
                                x: modelData.x - width / 2; y: modelData.y - height / 2
                                width: 26; height: 26; radius: 13; color: "#0b1119"
                                border.width: 1.5; border.color: modelData.active ? "#55d6a2" : "#f07878"
                                Text {
                                    anchors.centerIn: parent
                                    text: modelData.active ? "✓" : "✕"
                                    color: modelData.active ? "#55d6a2" : "#f07878"
                                    font.pixelSize: 15; font.bold: true
                                }
                            }
                        }
                        Rectangle {
                            x: 8
                            y: networkDiagram.height - height - 8
                            width: Math.min(networkDiagram.width * 0.60, 650)
                            height: 224
                            radius: 8
                            color: "#0d1721"
                            border.color: "#30485b"
                            Column {
                                anchors.fill: parent
                                anchors.margins: 9
                                spacing: 3
                                Row {
                                    width: parent.width
                                    height: 20
                                    spacing: 6
                                    Text { width: 125; text: "LINK"; color: "#829aad"; font.pixelSize: 10; font.bold: true }
                                    Text { width: 88; text: "STATE"; color: "#829aad"; font.pixelSize: 10; font.bold: true }
                                    Text { width: parent.width - 225; text: "PROTOCOL / LIVE STATS"; color: "#829aad"; font.pixelSize: 10; font.bold: true }
                                }
                                Rectangle { width: parent.width; height: 1; color: "#30485b" }
                                Row {
                                    width: parent.width; height: 34; spacing: 6
                                    Text { width: 125; text: "Station - router"; color: "#d5e3ec"; font.pixelSize: 11; anchors.verticalCenter: parent.verticalCenter }
                                    Text { width: 88; text: root.stationOnBuoyRouter() ? "ONLINE" : "OFFLINE"; color: root.wifiNetworkColor(root.stationOnBuoyRouter()); font.pixelSize: 11; font.bold: true; anchors.verticalCenter: parent.verticalCenter }
                                    Text { width: parent.width - 225; text: (backendObject ? backendObject.stationNetworkIp : "No IPv4") + "  |  Wi-Fi / Ethernet"; color: "#9eb5c7"; font.pixelSize: 11; elide: Text.ElideRight; anchors.verticalCenter: parent.verticalCenter }
                                }
                                Rectangle { width: parent.width; height: 1; color: "#1d2c3a" }
                                Row {
                                    width: parent.width; height: 46; spacing: 6
                                    Text { width: 125; text: "Router - buoy"; color: "#d5e3ec"; font.pixelSize: 11; anchors.verticalCenter: parent.verticalCenter }
                                    Text { width: 88; text: root.buoyControlStatus(); color: root.buoyControlStatusColor(); font.pixelSize: 11; font.bold: true; anchors.verticalCenter: parent.verticalCenter }
                                    Text {
                                        width: parent.width - 225
                                        text: "TCP " + (backendState.tcpPort || 5000) + " | UDP " + (backendState.udpPort || 5001) + "/" + (backendState.audioPort || 5002) + "\n" + (root.wifiBuoyConnected() ? (backendState.wifiReceivedMessages || 0) + " RX | " + Number(backendState.wifiMessageRate || 0).toFixed(1) + " msg/s | " + Number(backendState.wifiThroughputKib || 0).toFixed(1) + " KiB/s" : "No active buoy traffic")
                                        color: "#9eb5c7"; font.pixelSize: 11; elide: Text.ElideRight; anchors.verticalCenter: parent.verticalCenter
                                    }
                                }
                                Rectangle { width: parent.width; height: 1; color: "#1d2c3a" }
                                Row {
                                    width: parent.width; height: 34; spacing: 6
                                    Text { width: 125; text: "Router - Backpack"; color: "#d5e3ec"; font.pixelSize: 11; anchors.verticalCenter: parent.verticalCenter }
                                    Text { width: 88; text: backendState.radioWifiIp ? "CONNECTED" : "NOT FOUND"; color: root.wifiNetworkColor(Boolean(backendState.radioWifiIp)); font.pixelSize: 11; font.bold: true; anchors.verticalCenter: parent.verticalCenter }
                                    Text { width: parent.width - 225; text: backendState.radioWifiIp ? backendState.radioWifiIp + " | HTTP / MAVLink UDP" : "Backpack Wi-Fi discovery"; color: "#9eb5c7"; font.pixelSize: 11; elide: Text.ElideRight; anchors.verticalCenter: parent.verticalCenter }
                                }
                                Rectangle { width: parent.width; height: 1; color: "#1d2c3a" }
                                Row {
                                    width: parent.width; height: 34; spacing: 6
                                    Text { width: 125; text: "TX12 - buoy"; color: "#d5e3ec"; font.pixelSize: 11; anchors.verticalCenter: parent.verticalCenter }
                                    Text { width: 88; text: root.elrsLinkActive() ? "RF LIVE" : "NO DATA"; color: root.elrsLinkActive() ? "#55d6a2" : "#f07878"; font.pixelSize: 11; font.bold: true; anchors.verticalCenter: parent.verticalCenter }
                                    Text { width: parent.width - 225; text: "ELRS / MAVLink | " + root.elrsLinkDetails(); color: root.elrsLinkActive() ? "#9eb5c7" : "#829aad"; font.pixelSize: 11; elide: Text.ElideRight; anchors.verticalCenter: parent.verticalCenter }
                                }
                            }
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true; spacing: 9
                        Text { text: backendState.networkScanStatus || ""; color: "#9eb5c7"; font.pixelSize: 10; Layout.fillWidth: true; elide: Text.ElideRight }
                        Button { text: backendState.networkScanning ? "Scanning…" : "Scan buoy devices"; enabled: Boolean(backendObject) && !Boolean(backendState.networkScanning); onClicked: backendObject.scanBuoyNetwork() }
                    }
                    RowLayout {
                        Layout.fillWidth: true; spacing: 8
                        TextField { id: commandText; Layout.fillWidth: true; implicitHeight: 34; placeholderText: "Send a text command to the buoy"; onAccepted: root.sendCommand() }
                        Button { text: "Send"; enabled: commandText.text.trim().length > 0 && root.wifiBuoyConnected(); onClicked: root.sendCommand() }
                        Text { text: backendState.lastSendResult || ""; color: "#829aad"; font.pixelSize: 10; Layout.preferredWidth: 155; elide: Text.ElideRight }
                    }
                }
            }

            Rectangle {
                radius: 12; color: "#0b1119"; border.color: "#395166"
                ColumnLayout {
                    anchors.fill: parent; anchors.margins: 16; spacing: 8
                    Text { text: "RadioMaster TX12 / ELRS"; color: "#e6ecf2"; font.pixelSize: 23; font.bold: true }
                    Text { text: "Backpack Wi-Fi confirms the PC can reach the backpack. ELRS link quality appears only when the buoy receiver relays RADIO_STATUS over MAVLink."; color: "#829aad"; wrapMode: Text.WordWrap; Layout.fillWidth: true }
                    RowLayout {
                        Layout.fillWidth: true; spacing: 14
                        Item {
                            id: radioPhoto
                            Layout.preferredWidth: 290
                            Layout.preferredHeight: width * 1199 / 1312
                            Image {
                                anchors.fill: parent
                                source: "../assets/tx12-mkii-front-white.png"
                                fillMode: Image.PreserveAspectFit
                                smooth: true
                            }
                            Repeater {
                                model: root.displayedControls().filter(control => control.image_x !== null && control.image_y !== null)
                                delegate: Rectangle {
                                    required property var modelData
                                    width: 30; height: 19; radius: 6
                                    x: radioPhoto.width * (modelData.image_x || 0) - width / 2
                                    y: radioPhoto.height * (modelData.image_y || 0) - height / 2
                                    color: "#0c2936"; border.color: "#9ce8e3"
                                    Text { anchors.centerIn: parent; text: root.markerLabel(modelData); color: "#f2fbfc"; font.pixelSize: 9; font.bold: true }
                                }
                            }
                        }
                        ColumnLayout {
                            Layout.fillWidth: true; Layout.alignment: Qt.AlignTop; spacing: 4
                            Rectangle {
                                Layout.fillWidth: true; Layout.preferredHeight: 34; radius: 5; color: "#111c28"
                                RowLayout {
                                    anchors.fill: parent; anchors.leftMargin: 9; anchors.rightMargin: 6; spacing: 7
                                    Rectangle {
                                        width: 9; height: 9; radius: 5
                                        color: backendState.radioWifiIp ? "#55d6a2" : "#607080"
                                    }
                                    Text {
                                        text: backendState.radioWifiIp ? "Backpack Wi-Fi connected" : "Backpack Wi-Fi not found"
                                        color: backendState.radioWifiIp ? "#55d6a2" : "#9eb5c7"
                                        font.pixelSize: 11; font.bold: true
                                    }
                                    Text {
                                        text: backendState.radioWifiIp || (backendState.radioWifiStatus || "Not scanned")
                                        color: "#9eb5c7"; font.pixelSize: 10; Layout.fillWidth: true; elide: Text.ElideRight
                                    }
                                    Button {
                                        text: "Find"; enabled: !Boolean(backendState.radioWifiScanning)
                                        onClicked: if (backendObject) backendObject.findRadioBackpack()
                                    }
                                }
                            }
                            Rectangle {
                                id: linkQualityIndicator
                                property real percent: root.linkQualityPercent()
                                Layout.fillWidth: true; Layout.preferredHeight: 38; radius: 5; color: "#111c28"
                                RowLayout {
                                    anchors.fill: parent; anchors.leftMargin: 9; anchors.rightMargin: 9; spacing: 9
                                    Item {
                                        Layout.preferredWidth: 30; Layout.preferredHeight: 26
                                        Row {
                                            anchors.bottom: parent.bottom; anchors.horizontalCenter: parent.horizontalCenter; spacing: 3
                                            Repeater {
                                                model: 5
                                                delegate: Rectangle {
                                                    required property int index
                                                    width: 4; height: 7 + index * 4; radius: 2
                                                    color: linkQualityIndicator.percent >= 0 && linkQualityIndicator.percent >= (index + 1) * 20 ? root.linkQualityColor(linkQualityIndicator.percent) : "#354554"
                                                }
                                            }
                                        }
                                    }
                                    Text { text: "ELRS link quality"; color: "#e6ecf2"; font.pixelSize: 11; font.bold: true; Layout.fillWidth: true }
                                    Text {
                                        text: linkQualityIndicator.percent >= 0 ? Math.round(linkQualityIndicator.percent) + "%" : "No link data"
                                        color: root.linkQualityColor(linkQualityIndicator.percent); font.pixelSize: 13; font.bold: true
                                    }
                                }
                            }
                            RowLayout {
                                Layout.fillWidth: true
                                Text { text: "ELRS buoy telemetry"; color: "#e6ecf2"; font.pixelSize: 15; font.bold: true }
                                Text { text: backendState.radioTelemetryStatus || "Not listening"; color: "#829aad"; font.pixelSize: 10; Layout.fillWidth: true; horizontalAlignment: Text.AlignRight; elide: Text.ElideRight }
                            }
                            Repeater {
                                model: [
                                    { label: "System ID", value: backendState.radioSystemId || "Unknown" },
                                    { label: "Validated packets", value: String(backendState.radioTelemetryPackets || 0) },
                                    { label: "Radio status", value: backendState.radioLinkStatus || "No receiver RADIO_STATUS relayed" },
                                    { label: "RSSI", value: backendState.radioRssiStatus || "Unknown" },
                                    { label: "SNR", value: backendState.radioSnrStatus || "Unknown" }
                                ]
                                delegate: Rectangle {
                                    required property var modelData
                                    Layout.fillWidth: true; implicitHeight: 25; radius: 4
                                    color: "#111c28"
                                    RowLayout {
                                        anchors.fill: parent; anchors.leftMargin: 8; anchors.rightMargin: 8
                                        Text { text: modelData.label; color: "#9eb5c7"; font.pixelSize: 11; Layout.preferredWidth: 112 }
                                        Text { text: modelData.value; color: "#e6ecf2"; font.pixelSize: 11; Layout.fillWidth: true; elide: Text.ElideRight }
                                    }
                                }
                            }
                        }
                    }
                    RowLayout {
                        Layout.fillWidth: true; spacing: 8
                        Text { text: "TX12 control map"; color: "#e6ecf2"; font.pixelSize: 16; font.bold: true; Layout.fillWidth: true }
                        Text { text: backendState.tx12MapStatus || "Mapping not loaded"; color: "#829aad"; font.pixelSize: 10; elide: Text.ElideRight; Layout.maximumWidth: 290 }
                        StyledCheckBox { text: "Show unmapped"; checked: root.showUnmappedControls; onToggled: root.showUnmappedControls = checked }
                        Button { text: "Reload map"; onClicked: if (backendObject) backendObject.reloadTx12Mapping() }
                    }
                    Tx12MappingPanel { Layout.fillWidth: true; controls: root.displayedControls(); backendState: root.backendState }
                    Item { Layout.fillHeight: true }
                }
            }

            LogsPanel {
                backendObject: root.backendObject
                backendState: root.backendState
                active: sections.currentIndex === 2 && root.backendObject && root.backendObject.dashboardTab === "communications"
            }
        }
    }
}
