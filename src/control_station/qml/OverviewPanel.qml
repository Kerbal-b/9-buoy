import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    id: root
    property var backendObject
    property var backendState: ({})
    property var tileDefinitions: [
        { title: "Communications", tab: "communications" },
        { title: "Power", tab: "power" },
        { title: "Instruments", tab: "instrument_debug" },
        { title: "Movement", tab: "motors" },
        { title: "Navigation", tab: "navigation" },
        { title: "SD Card", tab: "sd_debug" }
    ]
    property var networkDeviceDefinitions: [
        { label: "STATION", source: "../assets/network-control-station.png" },
        { label: "ROUTER", source: "../assets/network-router.png" },
        { label: "BUOY", source: "../assets/network-buoy.png" },
        { label: "TX12 BP", source: "../assets/tx12-mkii-front-white.png" }
    ]

    function batteryPercent() {
        const match = String(backendState.batteryStatus || "").match(/(\d+(?:\.\d+)?)\s*%/)
        return match ? Math.round(Number(match[1])) + "%" : "Unknown"
    }

    function chargeStatus() {
        if (backendState.telemetrySource === "ELRS") return "ELRS battery current"
        const current = Number.parseFloat(String(backendState.currentDraw || ""))
        if (!Number.isFinite(current)) return "Charging unknown"
        if (current > 0.05) return "Charging"
        if (current < -0.05) return "Discharging"
        return "Current near zero"
    }

    function stationOnBuoyRouter() {
        const address = String(backendObject ? backendObject.stationNetworkIp : "")
        return /^192\.168\.8\.\d+$/.test(address)
    }

    function buoyConnected() {
        return String(backendState.serialStatus || "").startsWith("Connected") && String(backendState.serialTarget || "").startsWith("WIFI:")
    }

    function networkDeviceColor(label) {
        const foundBuoys = backendState.networkBuoyDevices || []
        if (label === "BUOY") return buoyConnected() ? "#55d6a2" : foundBuoys.length ? "#f2bf58" : "#536575"
        if (label === "TX12 BP") return backendState.radioWifiIp ? "#55d6a2" : "#536575"
        return stationOnBuoyRouter() ? "#55d6a2" : "#536575"
    }

    function communicationsValue() {
        if (!stationOnBuoyRouter()) return "Buoy router network not detected"
        return buoyConnected() ? "Router online - buoy connected" : "Router online - buoy link down"
    }

    function communicationsDetail() {
        const lq = String(backendState.radioLinkQuality || "No ELRS data")
        return (backendState.wifiReceivedMessages || 0) + " Wi-Fi RX messages | " + Number(backendState.wifiMessageRate || 0).toFixed(1) + " msg/s | " + Number(backendState.wifiThroughputKib || 0).toFixed(1) + " KiB/s | ELRS " + lq
    }

    function tileValue(title) {
        if (title === "Communications") return communicationsValue()
        if (title === "Power") return batteryPercent()
        if (title === "Instruments") return backendState.audioStreamEnabled ? "Audio streaming" : "Audio stream off"
        if (title === "Movement") return backendState.motorOutputTelemetry ? "Motor telemetry live" : "Motor telemetry unavailable"
        if (title === "Navigation") return backendState.currentLocation && backendState.currentLocation !== "Unknown" ? backendState.currentLocation : "No position fix"
        return backendState.sdCardCapacity && Number(backendState.sdCardCapacity.totalBytes) > 0 ? "Card detected" : "Capacity unknown"
    }

    function tileDetail(title) {
        if (title === "Communications") return communicationsDetail()
        if (title === "Power") return (backendState.telemetrySource || "No live telemetry") + "  |  " + chargeStatus() + "  |  " + (backendState.batteryStatus || "N/A")
        if (title === "Instruments") return "Depth " + (backendState.currentDepth || "N/A") + "  |  Water " + (backendState.waterTemperature || "N/A")
        if (title === "Movement") return "Lateral " + (backendState.turn || 0) + "%  |  Thrust " + (backendState.thrust || 0) + "%"
        if (title === "Navigation") return (backendState.telemetrySource || "No live telemetry") + "  |  " + (backendState.navigationStatus || "Navigation unavailable")
        return backendState.sdCardStatus || "Card status unavailable"
    }

    Rectangle {
        anchors.fill: parent
        radius: 12
        color: "#0b1119"
        border.color: "#395166"

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 22
            spacing: 16

            Text { text: "Overview"; color: "#e6ecf2"; font.pixelSize: 24; font.bold: true }
            Text { text: "Buoy systems at a glance. Select a tile for details."; color: "#829aad"; font.pixelSize: 13 }

            GridLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                columns: 2
                columnSpacing: 14
                rowSpacing: 14

                Repeater {
                    model: root.tileDefinitions

                    delegate: Rectangle {
                        required property var modelData
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        Layout.minimumHeight: 145
                        radius: 12
                        color: tileMouse.containsMouse ? "#1b3043" : "#111c28"
                        border.color: tileMouse.containsMouse ? "#50aaff" : "#395166"

                        Column {
                            anchors.fill: parent
                            anchors.margins: 14
                            spacing: 8
                            Text { text: modelData.title; color: "#80bfff"; font.pixelSize: 15; font.bold: true }
                            Item {
                                width: parent.width
                                height: visible ? 80 : 0
                                visible: modelData.title === "Communications"
                                RowLayout {
                                    anchors.fill: parent
                                    spacing: 8
                                    Repeater {
                                        model: root.networkDeviceDefinitions
                                        delegate: Item {
                                            required property var modelData
                                            Layout.fillWidth: true
                                            Layout.preferredWidth: 76
                                            Layout.fillHeight: true
                                            Image {
                                                anchors.horizontalCenter: parent.horizontalCenter
                                                anchors.top: parent.top
                                                width: 82; height: 62
                                                source: modelData.source
                                                sourceSize.width: 256
                                                fillMode: Image.PreserveAspectFit
                                                smooth: true
                                                mipmap: true
                                            }
                                            Rectangle {
                                                x: parent.width / 2 + 30; y: 1; width: 10; height: 10; radius: 5
                                                color: root.networkDeviceColor(modelData.label)
                                                border.color: "#0b1119"; border.width: 1
                                            }
                                            Text {
                                                anchors.bottom: parent.bottom; anchors.horizontalCenter: parent.horizontalCenter
                                                text: modelData.label; color: "#9eb5c7"; font.pixelSize: 10; font.bold: true
                                            }
                                        }
                                    }
                                }
                            }
                            Text { text: root.tileValue(modelData.title); width: parent.width; color: "#e6ecf2"; font.pixelSize: 19; font.bold: true; elide: Text.ElideRight }
                            Text { text: root.tileDetail(modelData.title); width: parent.width; color: "#9eb5c7"; font.pixelSize: 12; wrapMode: Text.WordWrap; maximumLineCount: 2; elide: Text.ElideRight }
                        }
                        MouseArea {
                            id: tileMouse
                            anchors.fill: parent
                            hoverEnabled: true
                            cursorShape: Qt.PointingHandCursor
                            onClicked: if (backendObject) backendObject.setDashboardTab(modelData.tab)
                        }
                    }
                }
            }
        }
    }
}
