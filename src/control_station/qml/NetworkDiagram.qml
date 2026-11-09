import QtQuick

Rectangle {
    id: root
    property var backendObject
    property var backendState: ({})
    property bool onBuoyNetwork: false
    property bool buoyWifiConnected: false
    property bool backpackFound: false
    property bool backpackTelemetry: false
    property bool rfObserved: false

    readonly property real nodeWidth: Math.min(174, width * 0.21)
    readonly property color online: "#58d7a5"
    readonly property color waiting: "#8295a5"

    function buoyAddress() {
        const target = String(backendState.serialTarget || "")
        if (target.startsWith("WIFI:")) {
            const address = target.substring(5).split(":")[0]
            if (address && address.toLowerCase() !== "auto") return address
        }
        const devices = backendState.networkBuoyDevices || []
        return devices.length ? String(devices[0].ip || "") : ""
    }

    implicitHeight: 285
    radius: 12
    color: "#101a26"
    border.color: "#3b5064"
    clip: true

    component NetworkNode: Rectangle {
        property string title: ""
        property string stateText: ""
        property string detail: ""
        property color accent: "#8295a5"
        width: root.nodeWidth
        height: 74
        radius: 10
        color: "#172534"
        border.color: accent
        border.width: 1.5
        Rectangle { x: 10; y: 12; width: 6; height: 6; radius: 3; color: accent }
        Text { x: 23; y: 10; width: parent.width - 33; text: title; color: "#e6ecf2"; font.pixelSize: 11; font.bold: true; elide: Text.ElideRight }
        Text { x: 10; y: 32; width: parent.width - 20; text: stateText; color: accent; font.pixelSize: 10; font.bold: true; elide: Text.ElideRight }
        Text { x: 10; y: 49; width: parent.width - 20; text: detail; color: "#91a8b9"; font.pixelSize: 9; elide: Text.ElideMiddle }
    }

    Canvas {
        anchors.fill: parent
        property bool stationState: root.onBuoyNetwork
        property bool buoyState: root.buoyWifiConnected
        property bool backpackState: root.backpackFound
        property bool radioState: root.backpackTelemetry
        onStationStateChanged: requestPaint()
        onBuoyStateChanged: requestPaint()
        onBackpackStateChanged: requestPaint()
        onRadioStateChanged: requestPaint()
        onWidthChanged: requestPaint()
        onHeightChanged: requestPaint()
        onPaint: {
            const ctx = getContext("2d")
            ctx.clearRect(0, 0, width, height)
            function link(x1, y1, x2, y2, active, color) {
                ctx.beginPath()
                ctx.moveTo(x1, y1)
                ctx.lineTo(x2, y2)
                ctx.lineWidth = active ? 2.5 : 1.5
                ctx.strokeStyle = active ? color : "#405365"
                ctx.stroke()
            }
            link(width * 0.17 + root.nodeWidth / 2, 76,
                 width * 0.50 - root.nodeWidth / 2, 76, root.onBuoyNetwork, root.online)
            link(width * 0.50 + root.nodeWidth / 2, 76,
                 width * 0.83 - root.nodeWidth / 2, 76, root.buoyWifiConnected, root.online)
            link(width * 0.50 + 12, 112,
                 width * 0.64 - 12, 178, root.backpackFound, root.online)
            link(width * 0.64 + root.nodeWidth / 2, 215,
                 width * 0.83, 113, root.backpackTelemetry, "#7abfff")
        }
    }

    NetworkNode {
        x: parent.width * 0.17 - width / 2; y: 39
        title: "CONTROL STATION"
        stateText: root.onBuoyNetwork ? "On buoy network" : "Network absent"
        detail: backendObject ? backendObject.stationNetworkIp : "No address"
        accent: root.onBuoyNetwork ? root.online : root.waiting
    }
    NetworkNode {
        x: parent.width * 0.50 - width / 2; y: 39
        title: "BUOY ROUTER"
        stateText: root.onBuoyNetwork ? "Network detected" : "Not detected"
        detail: "192.168.8.0/24"
        accent: root.onBuoyNetwork ? root.online : root.waiting
    }
    NetworkNode {
        x: parent.width * 0.83 - width / 2; y: 39
        title: "BUOY WI-FI"
        stateText: root.buoyWifiConnected ? "Connected" : (backendState.networkBuoyDevices || []).length ? "Reachable" : "No control link"
        detail: root.buoyAddress() ? root.buoyAddress() + (root.buoyWifiConnected ? " · " + Number(backendState.wifiMessageRate || 0).toFixed(1) + " msg/s" : "") : "TCP control + detail"
        accent: root.buoyWifiConnected ? root.online : (backendState.networkBuoyDevices || []).length ? "#f1bd68" : root.waiting
    }
    NetworkNode {
        x: parent.width * 0.64 - width / 2; y: 178
        title: "TX BACKPACK"
        stateText: root.backpackTelemetry ? "Buoy telemetry live" : root.backpackFound ? "Discovered" : "Not discovered"
        detail: root.backpackFound ? String(backendState.radioWifiIp) : "MAVLink forwarding"
        accent: root.backpackTelemetry ? root.online : root.backpackFound ? "#f1bd68" : root.waiting
    }

    Text { x: parent.width * 0.32 - width / 2; y: 55; text: "LOCAL NETWORK"; color: "#7f98aa"; font.pixelSize: 9; font.bold: true }
    Text { x: parent.width * 0.67 - width / 2; y: 55; text: "WI-FI CONTROL"; color: "#7f98aa"; font.pixelSize: 9; font.bold: true }
    Text { x: parent.width * 0.53; y: 146; text: "ROUTER WI-FI"; color: "#7f98aa"; font.pixelSize: 9; font.bold: true }
    Text { x: parent.width * 0.74; y: 180; text: "ELRS / MAVLINK"; color: "#7f98aa"; font.pixelSize: 9; font.bold: true }
    Text { anchors.left: parent.left; anchors.bottom: parent.bottom; anchors.margins: 12; text: root.rfObserved ? "RF observed  ·  " + backendState.radioLinkQuality : "RF status unverified"; color: root.rfObserved ? root.online : root.waiting; font.pixelSize: 10; font.bold: true }
}
