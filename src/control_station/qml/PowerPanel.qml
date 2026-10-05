import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Item {
    property var backendState: ({})

    Rectangle {
        anchors.fill: parent
        radius: 12
        color: "#0b1119"
        border.color: "#395166"

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 22
            spacing: 14
            Text { text: "Power"; color: "#e6ecf2"; font.pixelSize: 24; font.bold: true }
            Text { text: "Battery telemetry source: " + (backendState.telemetrySource || "No live telemetry"); color: "#829aad"; font.pixelSize: 13 }

            Repeater {
                model: [
                    { label: "Battery", value: backendState.batteryStatus || "N/A" },
                    { label: "Battery current", value: backendState.currentDraw || "N/A" },
                    { label: "Charging", value: chargingText() },
                    { label: "Solar input", value: "Telemetry unavailable" }
                ]
                delegate: Rectangle {
                    required property var modelData
                    Layout.fillWidth: true
                    implicitHeight: 62
                    radius: 9
                    color: "#111c28"
                    border.color: "#395166"
                    RowLayout {
                        anchors.fill: parent
                        anchors.margins: 16
                        Text { text: modelData.label; color: "#9eb5c7"; font.pixelSize: 14; Layout.fillWidth: true }
                        Text { text: modelData.value; color: "#e6ecf2"; font.pixelSize: 15; font.bold: true }
                    }
                }
            }
            Item { Layout.fillHeight: true }
        }
    }

    function chargingText() {
        if (backendState.telemetrySource === "ELRS") return "Direction unavailable from this MAVLink field"
        const current = Number.parseFloat(String(backendState.currentDraw || ""))
        if (!Number.isFinite(current)) return "Unknown"
        if (current > 0.05) return "Charging (positive battery current)"
        if (current < -0.05) return "Discharging (negative battery current)"
        return "Current near zero"
    }
}
