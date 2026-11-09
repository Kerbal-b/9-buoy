import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Item {
    id: root
    property var backendObject
    property var backendState: ({})
    property bool showUnmappedControls: false
    property bool showDiagnostics: false

    function hasMappedFunction(control) {
        if (control.channel === null || control.channel === undefined) return false
        if (control.purpose && control.purpose.trim() && control.purpose !== "Unassigned") return true
        return Boolean(control.positions && control.positions.some(position => position.meaning && position.meaning.trim()))
    }

    function displayedControls() {
        const controls = backendState.tx12Controls || []
        return showUnmappedControls ? controls : controls.filter(control => hasMappedFunction(control))
    }

    function linkQualityPercent() {
        const match = String(backendState.radioLinkQuality || "").match(/(\d+(?:\.\d+)?)%/)
        return match ? Math.max(0, Math.min(100, Number(match[1]))) : -1
    }

    function linkQualityColor() {
        const value = linkQualityPercent()
        if (value < 0) return "#8195a6"
        if (value >= 70) return "#58d7a5"
        if (value >= 40) return "#f1bd68"
        return "#f1787c"
    }

    function linkObserved() {
        return linkQualityPercent() >= 0 && !String(backendState.radioLinkStatus || "").toLowerCase().includes("stale")
    }

    function valueOrWaiting(value) {
        return value && String(value).trim() ? String(value) : "Waiting for telemetry"
    }

    function sessionSummary() {
        if (String(backendState.telemetrySource || "").toUpperCase() === "ELRS" && backendState.radioBuoyState) {
            if (!backendState.radioBuoyState.recording) return "SESSION IDLE"
            if (backendState.radioAudioRecording === true) return "SESSION + AUDIO"
            if (backendState.radioAudioRecording === false) return "SESSION ONLY"
            return "SESSION RECORDING"
        }
        if (!backendState.wifiDetailTelemetryLive && !backendState.wifiSafetyTelemetryLive) return "SESSION STATE WAITING"
        const state = String(backendState.scienceExperimentState || "UNKNOWN").toUpperCase()
        if (state === "RECORDING") return backendState.audioRecordingEnabled ? "SESSION + AUDIO" : "SESSION ONLY"
        if (state === "STOPPED") return "SAVE PENDING"
        if (state === "IDLE") return "SESSION IDLE"
        return "SESSION STATE WAITING"
    }

    function sessionColor() {
        const status = sessionSummary()
        if (status === "SESSION + AUDIO" || status === "SESSION ONLY" || status === "SESSION RECORDING") return "#58d7a5"
        if (status === "SAVE PENDING") return "#f1bd68"
        return "#91a8b9"
    }

    function signalReading(value) {
        const match = String(value || "").match(/-?\d+(?:\.\d+)?\s*dBm?\b/i)
        return match ? match[0] : "—"
    }

    component PanelButton: Button {
        id: panelButton
        property bool primary: false
        implicitHeight: 34
        implicitWidth: Math.max(45, buttonLabel.implicitWidth + 20)
        background: Rectangle {
            radius: 8
            color: !panelButton.enabled ? "#172431" : panelButton.primary ? "#175662" : panelButton.hovered ? "#263c4d" : "#1c2d3b"
            border.color: !panelButton.enabled ? "#263643" : panelButton.primary ? "#3c9ca7" : "#385064"
        }
        contentItem: Text {
            id: buttonLabel
            text: panelButton.text
            color: !panelButton.enabled ? "#657b8b" : panelButton.primary ? "#e8ffff" : "#dbe8ef"
            font.pixelSize: 11
            font.bold: true
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
        }
    }

    ScrollView {
        id: radioScroll
        anchors.fill: parent
        clip: true
        contentWidth: availableWidth
        contentHeight: radioContent.implicitHeight + 32
        ScrollBar.vertical.policy: ScrollBar.AsNeeded

        Column {
            id: radioContent
            x: 16
            y: 16
            width: Math.max(0, radioScroll.availableWidth - 32)
            spacing: 14

            RowLayout {
                width: parent.width
                spacing: 12
                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 3
                    Text { text: "Radio / ELRS"; color: "#e6ecf2"; font.pixelSize: 23; font.bold: true }
                    Text { text: "TX12 controls, receiver link, and telemetry"; color: "#91a8b9"; font.pixelSize: 12 }
                }
                RowLayout {
                    spacing: 7
                    Rectangle { Layout.preferredWidth: 8; Layout.preferredHeight: 8; radius: 4; color: root.linkObserved() ? "#58d7a5" : "#8295a5" }
                    Text { text: root.linkObserved() ? "RF observed" : "RF unverified"; color: root.linkObserved() ? "#9ce8c5" : "#afc0cc"; font.pixelSize: 11; font.bold: true }
                }
            }

            GridLayout {
                width: parent.width
                columns: width >= 820 ? 3 : 1
                columnSpacing: 12
                rowSpacing: 12

                Rectangle {
                    Layout.fillWidth: true; Layout.preferredHeight: 155
                    radius: 12; color: "#121c28"; border.color: "#3b5064"
                    Column {
                        anchors.fill: parent; anchors.margins: 16; spacing: 7
                        Text { text: "LINK QUALITY"; color: "#7abfff"; font.pixelSize: 11; font.bold: true }
                        Text { text: root.linkQualityPercent() >= 0 ? Math.round(root.linkQualityPercent()) + "%" : "—"; color: root.linkQualityColor(); font.pixelSize: 37; font.bold: true }
                        Rectangle {
                            width: parent.width; height: 10; radius: 5; color: "#263747"
                            Rectangle { width: parent.width * Math.max(0, root.linkQualityPercent()) / 100; height: parent.height; radius: 5; color: root.linkQualityColor() }
                        }
                        Text { width: parent.width; text: root.valueOrWaiting(backendState.radioLinkStatus); color: "#91a8b9"; font.pixelSize: 11; elide: Text.ElideRight }
                    }
                }

                Rectangle {
                    Layout.fillWidth: true; Layout.preferredHeight: 155
                    radius: 12; color: "#121c28"; border.color: "#3b5064"; clip: true
                    Column {
                        anchors.fill: parent; anchors.margins: 16; spacing: 8
                        Text { text: "SIGNAL"; color: "#7abfff"; font.pixelSize: 11; font.bold: true }
                        RowLayout {
                            width: parent.width; spacing: 12
                            ColumnLayout {
                                Layout.fillWidth: true; Layout.minimumWidth: 0; spacing: 3
                                Text { text: "RSSI"; color: "#8fa6b8"; font.pixelSize: 11 }
                                Text { text: root.signalReading(backendState.radioRssiStatus); color: "#e6ecf2"; font.pixelSize: 18; font.bold: true; Layout.fillWidth: true; Layout.minimumWidth: 0; elide: Text.ElideRight }
                            }
                            ColumnLayout {
                                Layout.fillWidth: true; Layout.minimumWidth: 0; spacing: 3
                                Text { text: "SNR"; color: "#8fa6b8"; font.pixelSize: 11 }
                                Text { text: root.signalReading(backendState.radioSnrStatus); color: "#e6ecf2"; font.pixelSize: 18; font.bold: true; Layout.fillWidth: true; Layout.minimumWidth: 0; elide: Text.ElideRight }
                            }
                        }
                        Text { width: parent.width; text: "Receiver measurements relayed by the buoy"; color: "#91a8b9"; font.pixelSize: 11; wrapMode: Text.WordWrap }
                    }
                }

                Rectangle {
                    Layout.fillWidth: true; Layout.preferredHeight: 155
                    radius: 12; color: "#121c28"; border.color: "#3b5064"
                    Column {
                        anchors.fill: parent; anchors.margins: 16; spacing: 7
                        Text { text: "CONTROL AUTHORITY"; color: "#7abfff"; font.pixelSize: 11; font.bold: true }
                        Text { width: parent.width; text: root.valueOrWaiting(backendState.controlSource); color: "#e6ecf2"; font.pixelSize: 19; font.bold: true; elide: Text.ElideRight }
                        Text { width: parent.width; text: "TX12 arm  ·  " + root.valueOrWaiting(backendState.radioArmStatus); color: "#9ce8c5"; font.pixelSize: 12; elide: Text.ElideRight }
                        Text { width: parent.width; text: "Buoy mode  ·  " + root.valueOrWaiting(backendState.radioBuoyMode); color: "#91a8b9"; font.pixelSize: 11; elide: Text.ElideRight }
                        Text { width: parent.width; text: root.sessionSummary(); color: root.sessionColor(); font.pixelSize: 11; font.bold: true; elide: Text.ElideRight }
                    }
                }
            }

            RowLayout {
                width: parent.width; spacing: 8
                Text { text: "TX12 controls"; color: "#e6ecf2"; font.pixelSize: 17; font.bold: true; Layout.fillWidth: true }
                StyledCheckBox { text: "Show unmapped"; checked: root.showUnmappedControls; onToggled: root.showUnmappedControls = checked }
                PanelButton { text: "Reload map"; enabled: Boolean(root.backendObject); onClicked: root.backendObject.reloadTx12Mapping() }
            }
            Text { width: parent.width; text: backendState.tx12MapStatus || "Mapping not loaded"; color: "#91a8b9"; font.pixelSize: 11; elide: Text.ElideRight }
            Tx12MappingPanel { width: parent.width; controls: root.displayedControls(); backendState: root.backendState }

            Rectangle {
                width: parent.width
                height: diagnosticsColumn.implicitHeight + 28
                radius: 12; color: "#121c28"; border.color: "#3b5064"
                Column {
                    id: diagnosticsColumn
                    x: 14; y: 14; width: parent.width - 28; spacing: 9
                    RowLayout {
                        width: parent.width
                        Text { text: "Telemetry details"; color: "#e6ecf2"; font.pixelSize: 14; font.bold: true; Layout.fillWidth: true }
                        PanelButton { text: root.showDiagnostics ? "Hide details" : "Show details"; onClicked: root.showDiagnostics = !root.showDiagnostics }
                    }
                    GridLayout {
                        visible: root.showDiagnostics
                        width: parent.width
                        columns: width >= 700 ? 2 : 1
                        columnSpacing: 16; rowSpacing: 8
                        Repeater {
                            model: [
                                { label: "System ID", value: backendState.radioSystemId },
                                { label: "Validated packets", value: String(backendState.radioTelemetryPackets || 0) },
                                { label: "Telemetry listener", value: backendState.radioTelemetryStatus },
                                { label: "Radio status", value: backendState.radioLinkStatus },
                                { label: "TX12 arm", value: backendState.radioArmStatus },
                                { label: "Manual assist", value: backendState.radioAssistMode },
                                { label: "Calibration", value: backendState.radioCalibrationStatus },
                                { label: "Calibration step", value: String(backendState.radioCalibrationStep || 0) +
                                    " at " + String(backendState.radioCalibrationPower || 0) + "% (limit " +
                                    String(backendState.radioCalibrationMaxPower || 0) + "%, " +
                                    String(backendState.radioCalibrationDuration || 0) + " s)" }
                            ]
                            delegate: RowLayout {
                                required property var modelData
                                Layout.fillWidth: true
                                Text { text: modelData.label; color: "#8fa6b8"; font.pixelSize: 11; Layout.preferredWidth: 120 }
                                Text { text: root.valueOrWaiting(modelData.value); color: "#dce7ef"; font.pixelSize: 11; Layout.fillWidth: true; elide: Text.ElideRight }
                            }
                        }
                    }
                }
            }
            Text {
                width: parent.width
                text: "Backpack page scope and update timing"
                color: "#e6ecf2"; font.pixelSize: 14; font.bold: true
            }
            Text {
                width: parent.width
                text: "With CH6 HIGH, buoy state and session/audio state arrive through Backpack MAVLink about once per second; changed channel banks arrive up to twice per second; battery/current and GPS about every 5 seconds; rotating science values about every 4 seconds. Science values go stale after 12 seconds and Backpack telemetry is marked stale after 5 seconds without packets. With CH6 LOW/MIDDLE, receiver link and channel data may instead arrive through buoy Wi-Fi; MAVLink to the Backpack is heartbeat-only."
                color: "#91a8b9"; font.pixelSize: 11; wrapMode: Text.WordWrap
            }
            Text {
                width: parent.width
                text: "If CH6 is HIGH and Backpack packets are missing, Backpack-delivered state and measurements are unavailable; Wi-Fi may still provide compact safety status when connected. If CH6 is LOW/MIDDLE, connect this station to buoy Wi-Fi for detailed telemetry. With neither Backpack packets nor buoy Wi-Fi, no telemetry reaches the station, although TX12 control may still work locally. Audio streaming, IMU streaming, and SD card actions are Wi-Fi-only."
                color: "#829aad"; font.pixelSize: 11; wrapMode: Text.WordWrap
            }
        }
    }
}
