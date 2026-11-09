import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Item {
    id: root
    property var backendObject
    property var backendState: ({})

    readonly property var buoy: backendState.radioBuoyState || null
    readonly property bool backpackSelected: String(backendState.telemetryMode || "WIFI").toUpperCase() === "MAVLINK"
    readonly property bool backpackLive: Boolean(buoy)
    readonly property bool wifiDetailLive: Boolean(backendState.wifiDetailTelemetryLive)
    readonly property bool wifiSafetyLive: Boolean(backendState.wifiSafetyTelemetryLive)
    readonly property string displaySource: backpackSelected
        ? backpackLive ? "BACKPACK" : wifiDetailLive ? "WI-FI" : wifiSafetyLive ? "WI-FI SAFETY" : "NONE"
        : wifiDetailLive ? "WI-FI" : backpackLive ? "BACKPACK" : wifiSafetyLive ? "WI-FI SAFETY" : "NONE"
    readonly property bool usingBackpack: displaySource === "BACKPACK"
    readonly property bool usingWifiDetail: displaySource === "WI-FI"
    readonly property bool usingWifiSafety: displaySource === "WI-FI SAFETY"
    readonly property bool rfObserved: /\d+%/.test(String(backendState.radioLinkQuality || ""))
        && !String(backendState.radioLinkStatus || "").toLowerCase().includes("stale")
        && (usingBackpack
            ? !String(backendState.radioLinkStatus || "").toLowerCase().includes("via buoy wi-fi")
            : usingWifiDetail && String(backendState.radioLinkStatus || "").toLowerCase().includes("via buoy wi-fi"))
    readonly property color green: "#58d7a5"
    readonly property color muted: "#91a8b9"

    function value(value, fallback) {
        const text = String(value === undefined || value === null ? "" : value).trim()
        return text && text !== "N/A" && text !== "Unknown" ? text : fallback
    }

    function batterySummary(rawValue) {
        const raw = String(rawValue || "")
        const voltage = raw.match(/\d+(?:\.\d+)?\s*V/i)
        const percent = raw.match(/\d+(?:\.\d+)?\s*%/)
        return [voltage ? voltage[0] : "", percent ? percent[0] : ""].filter(Boolean).join("  ·  ") || "Waiting"
    }

    function positionSummary(rawValue) {
        const position = String(rawValue || "")
        return position && position !== "Unknown" ? position : "Waiting for GPS"
    }

    function modeSummary() {
        if (usingBackpack) return value(buoy.mode, "Unknown mode")
        if (usingWifiDetail) return value(backendState.buoyMode, "Unknown mode").toUpperCase()
        if (usingWifiSafety) return "Safety status"
        return "Telemetry waiting"
    }

    function armSummary() {
        if (usingBackpack) return buoy.armed ? "Armed" : "Disarmed"
        if (usingWifiDetail || usingWifiSafety) return value(backendState.radioArmStatus, "Arm unknown")
        return "No fresh buoy state"
    }

    function controlSummary() {
        if (usingBackpack) return value(buoy.control_source, "No control")
        if (usingWifiDetail || usingWifiSafety) return value(backendState.controlSource, "No control")
        return ""
    }

    function motionSummary() {
        if (usingBackpack) return value(backendState.radioGroundMotion, "Waiting for GPS motion")
        if (usingWifiDetail && backendState.navigationGpsSpeedAvailable) {
            const speed = Number(backendState.navigationGpsSpeedMps || 0).toFixed(2) + " m/s"
            return backendState.navigationGpsCourseAvailable
                ? speed + " · " + Math.round(Number(backendState.navigationGpsCourseDeg || 0)) + "°"
                : speed
        }
        return usingWifiSafety ? "Unavailable on Wi-Fi safety" : "Waiting for GPS motion"
    }

    function recoverySummary() {
        if (usingBackpack) return buoy.recovery_stop ? "STOP ACTIVE" : "Ready"
        if (usingWifiDetail || usingWifiSafety) return value(backendState.recoveryStatus, "Unknown")
        return "Unknown"
    }

    function recoveryAlert() {
        return usingBackpack ? Boolean(buoy.recovery_stop)
            : (usingWifiDetail || usingWifiSafety) && backendState.recoveryStatus === "MANUAL_STOP"
    }

    function sessionSummary() {
        if (usingBackpack) {
            if (!buoy.recording) return "SESSION IDLE"
            if (backendState.radioAudioRecording === true) return "RECORDING + AUDIO"
            if (backendState.radioAudioRecording === false) return "SESSION ONLY"
            return "SESSION RECORDING"
        }
        if (usingWifiDetail || usingWifiSafety) {
            const state = String(backendState.scienceExperimentState || "UNKNOWN").toUpperCase()
            if (state === "RECORDING") return backendState.audioRecordingEnabled ? "RECORDING + AUDIO" : "SESSION ONLY"
            if (state === "STOPPED") return "SAVE PENDING"
            if (state === "IDLE") return "SESSION IDLE"
        }
        return "STATE WAITING"
    }

    function sessionColor() {
        const status = sessionSummary()
        if (status === "SESSION ONLY" || status === "RECORDING + AUDIO" || status === "SESSION RECORDING") return green
        if (status === "SAVE PENDING") return "#f1bd68"
        return muted
    }

    function homeSummary() {
        const status = String(backendState.activeHomeStatus || "")
        if (status === "Manual Home") return "SET - MANUAL"
        if (status === "Mission Home") return "SET - MISSION"
        if (status === "No active Home") return "NOT SET"
        return "Waiting for telemetry"
    }

    function homeDistanceSummary() {
        if (homeSummary().indexOf("SET") !== 0) return "No active Home"
        const distance = Number(backendState.homeDistanceMeters)
        return Number.isFinite(distance) ? distance.toFixed(1) + " m" : "Waiting for GPS"
    }

    component StatusRow: Rectangle {
        property string label: ""
        property string reading: ""
        property color readingColor: "#e6ecf2"
        Layout.fillWidth: true
        Layout.preferredHeight: 38
        radius: 7
        color: "#101b27"
        border.color: "#2d4254"
        RowLayout {
            anchors.fill: parent
            anchors.leftMargin: 10
            anchors.rightMargin: 10
            spacing: 6
            Text { text: label; color: root.muted; font.pixelSize: 11; Layout.preferredWidth: 69 }
            Text { text: reading; color: readingColor; font.pixelSize: 11; font.bold: true; elide: Text.ElideRight; horizontalAlignment: Text.AlignRight; Layout.fillWidth: true; Layout.minimumWidth: 0 }
        }
    }

    component SideButton: Button {
        id: button
        property bool selected: false
        implicitHeight: 34
        background: Rectangle {
            radius: 8
            color: button.selected ? "#175662" : button.hovered ? "#263c4d" : "#1c2d3b"
            border.color: button.selected ? "#3c9ca7" : "#385064"
        }
        contentItem: Text {
            text: button.text
            color: "#e6ecf2"
            font.pixelSize: 11
            font.bold: true
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
        }
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 10

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 120
            radius: 12
            color: "#182230"
            border.color: "#41556b"
            RowLayout {
                anchors.fill: parent
                anchors.margins: 13
                spacing: 10
                Rectangle {
                    Layout.preferredWidth: 72
                    Layout.preferredHeight: 72
                    radius: 11
                    color: "#101b27"
                    Rectangle { anchors.centerIn: parent; width: 42; height: 42; radius: 21; color: "transparent"; border.color: "#46687e"; border.width: 2 }
                    Rectangle { anchors.centerIn: parent; width: 15; height: 15; radius: 8; color: root.displaySource !== "NONE" ? root.green : "#607789" }
                }
                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 4
                    Text { text: "BUOY"; color: root.muted; font.pixelSize: 10; font.bold: true }
                    Text { text: root.modeSummary(); color: "#e6ecf2"; font.pixelSize: 17; font.bold: true; elide: Text.ElideRight; Layout.fillWidth: true }
                    Text { text: root.armSummary() + (root.controlSummary() ? "  ·  " + root.controlSummary() : ""); color: root.displaySource !== "NONE" ? root.green : root.muted; font.pixelSize: 10; elide: Text.ElideRight; Layout.fillWidth: true }
                }
            }
            Rectangle {
                anchors.left: parent.left; anchors.right: parent.right; anchors.bottom: parent.bottom
                anchors.leftMargin: 13; anchors.rightMargin: 13; anchors.bottomMargin: 10
                height: 1; color: "#304559"
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            radius: 12
            color: "#182230"
            border.color: "#41556b"
            ScrollView {
                anchors.fill: parent
                anchors.margins: 12
                clip: true
                contentWidth: availableWidth
                ScrollBar.vertical.policy: ScrollBar.AsNeeded
                ColumnLayout {
                    width: parent.width
                    spacing: 8
                    Text { text: "LIVE TELEMETRY"; color: "#e6ecf2"; font.pixelSize: 12; font.bold: true }
                    Text { text: root.displaySource === "NONE" ? "Waiting for buoy data" : "Source: " + root.displaySource; color: root.displaySource === "NONE" ? root.muted : root.green; font.pixelSize: 11; font.bold: true }
                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: routeText.implicitHeight + 18
                        radius: 8
                        color: root.backpackSelected ? "#142535" : "#13302b"
                        border.color: root.backpackSelected ? "#4a7291" : "#3c9c7f"
                        Text {
                            id: routeText
                            anchors.left: parent.left; anchors.right: parent.right; anchors.top: parent.top
                            anchors.margins: 9
                            text: root.backpackSelected
                                ? "SF / CH6 HIGH · MAVLink → Backpack\nPower, GPS and motor telemetry use Backpack. Buoy Wi-Fi carries safety status."
                                : "SF / CH6 LOW · Buoy Wi-Fi\nDetailed telemetry uses the buoy Wi-Fi link."
                            color: "#dce8ef"
                            font.pixelSize: 10
                            wrapMode: Text.WordWrap
                        }
                    }
                    Text {
                        visible: root.backpackSelected && !root.backpackLive
                        text: "Backpack data missing. Wi-Fi safety status is shown when available; set SF/CH6 LOW for full Wi-Fi telemetry."
                        color: "#f1bd68"
                        font.pixelSize: 10
                        wrapMode: Text.WordWrap
                        Layout.fillWidth: true
                    }
                    StatusRow { label: "RF link"; reading: root.rfObserved ? root.value(backendState.radioLinkQuality, "Observed") : "Unverified"; readingColor: root.rfObserved ? root.green : root.muted }
                    StatusRow { label: "Battery"; reading: root.usingBackpack ? root.batterySummary(backendState.radioBatteryStatus) : root.usingWifiDetail ? root.batterySummary(backendState.batteryStatus) : "Unavailable" }
                    StatusRow { label: "Position"; reading: root.usingBackpack ? root.positionSummary(backendState.radioLocation) : root.usingWifiDetail ? root.positionSummary(backendState.currentLocation) : "Unavailable" }
                    StatusRow { label: "Home"; reading: root.homeSummary(); readingColor: root.homeSummary().indexOf("SET") === 0 ? root.green : root.homeSummary() === "NOT SET" ? "#f1bd68" : root.muted }
                    StatusRow { label: "To Home"; reading: root.homeDistanceSummary(); readingColor: root.homeSummary().indexOf("SET") === 0 ? "#e6ecf2" : root.muted }
                    StatusRow { label: "Motion"; reading: root.motionSummary() }
                    StatusRow { label: "Recovery"; reading: root.recoverySummary(); readingColor: root.recoveryAlert() ? "#f07878" : "#e6ecf2" }
                    StatusRow { label: "Session"; reading: root.sessionSummary(); readingColor: root.sessionColor() }
                    Text { text: "CONTROL INPUT"; color: root.muted; font.pixelSize: 10; font.bold: true; Layout.topMargin: 6 }
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 5
                        SideButton { text: "TX12"; selected: backendState.controlInputMode === "tx12"; Layout.fillWidth: true; onClicked: if (root.backendObject) root.backendObject.setControlInputMode("tx12") }
                        SideButton { text: "Keyboard"; selected: backendState.controlInputMode === "keyboard"; Layout.fillWidth: true; onClicked: if (root.backendObject) root.backendObject.setControlInputMode("keyboard") }
                    }
                    Text { text: "Mission route and point progress require the buoy Wi-Fi link."; color: root.muted; font.pixelSize: 10; wrapMode: Text.WordWrap; Layout.fillWidth: true }
                }
            }
        }
    }
}
