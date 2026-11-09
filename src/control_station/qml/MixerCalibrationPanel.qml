import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Dialogs
import QtQuick.Layouts

Item {
    id: root
    property var backendObject
    property var backendState: ({})
    property var candidateRows: []
    property string syncedRowsJson: ""
    readonly property var axisLabels: ["Forward", "Reverse", "Strafe right", "Strafe left", "Yaw clockwise", "Yaw counter-clockwise"]
    readonly property string ink: "#e7eef4"
    readonly property string muted: "#98adbd"
    readonly property string card: "#1d2b3a"
    readonly property string edge: "#354b5e"
    readonly property string green: "#72d7ad"
    readonly property bool connected: String(backendState.serialStatus || "").startsWith("Connected")
    readonly property bool disarmed: String(backendState.radioArmStatus || "").toUpperCase() === "DISARMED"
    readonly property int acceptedCount: {
        let count = 0
        for (let i = 0; i < candidateRows.length; ++i) if (candidateRows[i].set) ++count
        return count
    }

    function cloneRows(rows) { return JSON.parse(JSON.stringify(rows || [])) }
    function metric(value, digits, suffix) {
        return value === null || value === undefined ? "—" : Number(value).toFixed(digits) + suffix
    }
    FileDialog {
        id: candidateJsonDialog
        title: "Upload processed mixer calibration"
        fileMode: FileDialog.OpenFile
        nameFilters: ["Processed mixer profile (*.mixer-candidate.json *.json)", "All files (*)"]
        onAccepted: root.backendObject.applyMixerCalibrationCandidateJson(selectedFile.toString())
    }

    Component.onCompleted: {
        const rows = backendState.mixerCalibrationRows || []
        candidateRows = cloneRows(rows)
        syncedRowsJson = JSON.stringify(rows)
    }

    Connections {
        target: root.backendObject || null
        function onStateChanged() {
            const incoming = root.backendObject.state.mixerCalibrationRows || []
            const signature = JSON.stringify(incoming)
            if (signature !== root.syncedRowsJson) {
                root.candidateRows = root.cloneRows(incoming)
                root.syncedRowsJson = signature
            }
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 12
        spacing: 8
        RowLayout {
            Layout.fillWidth: true
            ColumnLayout {
                Layout.fillWidth: true; spacing: 1
                Text { text: "Empirical mixer profile"; color: root.ink; font.pixelSize: 21; font.bold: true }
                Text { text: "Three signed motor commands for six movements at each 10% input level"; color: root.muted; font.pixelSize: 10 }
            }
            Rectangle {
                radius: 12; color: root.acceptedCount === 60 ? "#173b34" : "#3b3024"
                implicitWidth: countLabel.implicitWidth + 18; implicitHeight: 27
                Text { id: countLabel; anchors.centerIn: parent; text: root.acceptedCount + " / 60 POINTS"; color: root.acceptedCount === 60 ? root.green : "#f2bb74"; font.pixelSize: 10; font.bold: true }
            }
        }

        RowLayout {
            Layout.fillWidth: true; spacing: 10
            Button {
                text: "Upload processed mixer profile"
                enabled: root.connected && root.disarmed
                onClicked: candidateJsonDialog.open()
            }
            Text {
                Layout.fillWidth: true
                text: root.connected
                    ? (root.disarmed ? "Choose the AI-processed .mixer-candidate.json to load the table and save it to buoy SD" : "Disarm the buoy before uploading a mixer profile")
                    : "Connect to the buoy before uploading a mixer profile"
                color: root.muted; font.pixelSize: 9; elide: Text.ElideRight
            }
        }

        RowLayout {
            Layout.fillWidth: true; spacing: 10
            Text {
                Layout.fillWidth: true
                text: backendState.mixerCalibrationStatus || "Upload an AI-processed mixer profile to populate this table and update the buoy"
                color: String(backendState.mixerCalibrationStatus || "").toLowerCase().includes("reject") || String(backendState.mixerCalibrationStatus || "").toLowerCase().includes("failed") ? "#ef8c87" : root.muted
                font.pixelSize: 10; elide: Text.ElideRight
            }
            Text { text: backendState.mixerCalibrationProfileActive ? "SD PROFILE ACTIVE" : "SAFE DEFAULT MIXER ACTIVE"; color: backendState.mixerCalibrationProfileActive ? root.green : "#f2bb74"; font.pixelSize: 9; font.bold: true }
            Text { text: backendState.mixerCalibrationCrc ? "CRC " + backendState.mixerCalibrationCrc : ""; color: root.muted; font.pixelSize: 9 }
        }
        Rectangle {
            Layout.fillWidth: true; Layout.fillHeight: true
            radius: 12; color: root.card; border.color: root.edge; clip: true
            ColumnLayout {
                anchors.fill: parent; anchors.margins: 8; spacing: 0
                RowLayout {
                    Layout.fillWidth: true; height: 28; spacing: 5
                    Text { text: "MOVEMENT"; Layout.preferredWidth: 128; color: root.muted; font.pixelSize: 9; font.bold: true }
                    Text { text: "INPUT"; Layout.preferredWidth: 44; color: root.muted; font.pixelSize: 9; font.bold: true }
                    Text { text: "SET"; Layout.preferredWidth: 48; color: root.muted; font.pixelSize: 9; font.bold: true }
                    Text { text: "REAR %"; Layout.preferredWidth: 104; color: root.muted; font.pixelSize: 9; font.bold: true }
                    Text { text: "FRONT LEFT %"; Layout.preferredWidth: 104; color: root.muted; font.pixelSize: 9; font.bold: true }
                    Text { text: "FRONT RIGHT %"; Layout.preferredWidth: 104; color: root.muted; font.pixelSize: 9; font.bold: true }
                    Text { text: "LOG N"; Layout.preferredWidth: 40; color: root.muted; font.pixelSize: 9; font.bold: true }
                    Text { text: "ACCEL X / Y"; Layout.preferredWidth: 105; color: root.muted; font.pixelSize: 9; font.bold: true }
                    Text { text: "GYRO Z"; Layout.preferredWidth: 75; color: root.muted; font.pixelSize: 9; font.bold: true }
                    Text { text: "CURRENT"; Layout.preferredWidth: 75; color: root.muted; font.pixelSize: 9; font.bold: true }
                    Text { text: "ACTUAL DRIVE / PWM · R / FL / FR"; Layout.preferredWidth: 190; color: root.muted; font.pixelSize: 9; font.bold: true }
                    Text { text: "GYRO X / Y"; Layout.preferredWidth: 90; color: root.muted; font.pixelSize: 9; font.bold: true }
                    Text { text: "HEADING"; Layout.preferredWidth: 62; color: root.muted; font.pixelSize: 9; font.bold: true }
                    Text { text: "VOLTAGE"; Layout.preferredWidth: 65; color: root.muted; font.pixelSize: 9; font.bold: true }
                    Text { text: "GPS SPEED / COURSE"; Layout.preferredWidth: 115; color: root.muted; font.pixelSize: 9; font.bold: true }
                    Text { text: "RAW MAG X / Y / Z"; Layout.preferredWidth: 125; color: root.muted; font.pixelSize: 9; font.bold: true }
                    Item { Layout.fillWidth: true }
                }
                Flickable {
                    id: pointView
                    Layout.fillWidth: true; Layout.fillHeight: true
                    contentWidth: Math.max(width, 1660); contentHeight: pointsColumn.implicitHeight
                    clip: true
                    ScrollBar.vertical: ScrollBar { }
                    ScrollBar.horizontal: ScrollBar { }
                    ColumnLayout {
                        id: pointsColumn
                        width: pointView.contentWidth; spacing: 2
                        Repeater {
                            model: root.candidateRows
                            delegate: Rectangle {
                                required property int index
                                required property var modelData
                                Layout.fillWidth: true; height: 37; radius: 5
                                color: modelData.set ? "#203b42" : (index % 2 ? "#172634" : "#1a2937")
                                RowLayout {
                                    anchors.fill: parent; anchors.leftMargin: 5; anchors.rightMargin: 5; spacing: 5
                                    Text { text: root.axisLabels[modelData.axis_index]; Layout.preferredWidth: 128; color: root.ink; font.pixelSize: 10; elide: Text.ElideRight }
                                    Text { text: modelData.power + "%"; Layout.preferredWidth: 44; color: root.ink; font.pixelSize: 10; font.bold: true }
                                    Text { Layout.preferredWidth: 48; text: modelData.set ? "YES" : "—"; color: modelData.set ? root.green : root.muted; horizontalAlignment: Text.AlignHCenter; font.pixelSize: 10 }
                                    Text { Layout.preferredWidth: 104; text: String(modelData.rear); color: root.ink; horizontalAlignment: Text.AlignHCenter; font.pixelSize: 10 }
                                    Text { Layout.preferredWidth: 104; text: String(modelData.front_left); color: root.ink; horizontalAlignment: Text.AlignHCenter; font.pixelSize: 10 }
                                    Text { Layout.preferredWidth: 104; text: String(modelData.front_right); color: root.ink; horizontalAlignment: Text.AlignHCenter; font.pixelSize: 10 }
                                    Text { text: modelData.samples ? String(modelData.samples) : "—"; Layout.preferredWidth: 40; color: root.muted; font.pixelSize: 10; horizontalAlignment: Text.AlignHCenter }
                                    Text { text: root.metric(modelData.accel_x_g, 3, "") + " / " + root.metric(modelData.accel_y_g, 3, "") + " g"; Layout.preferredWidth: 105; color: root.muted; font.pixelSize: 9; elide: Text.ElideRight }
                                    Text { text: root.metric(modelData.yaw_rate_dps, 1, " °/s"); Layout.preferredWidth: 75; color: root.muted; font.pixelSize: 9 }
                                    Text { text: root.metric(modelData.current_a, 2, " A"); Layout.preferredWidth: 75; color: root.muted; font.pixelSize: 9 }
                                    Text {
                                        text: "R " + root.metric(modelData.actual_drive_rear, 0, "") + "/" + root.metric(modelData.applied_pwm_rear, 0, "")
                                            + "  FL " + root.metric(modelData.actual_drive_front_left, 0, "") + "/" + root.metric(modelData.applied_pwm_front_left, 0, "")
                                            + "  FR " + root.metric(modelData.actual_drive_front_right, 0, "") + "/" + root.metric(modelData.applied_pwm_front_right, 0, "")
                                        Layout.preferredWidth: 190; color: root.muted; font.pixelSize: 9; elide: Text.ElideRight
                                    }
                                    Text { text: root.metric(modelData.gyro_x_dps, 1, "") + " / " + root.metric(modelData.gyro_y_dps, 1, "") + " °/s"; Layout.preferredWidth: 90; color: root.muted; font.pixelSize: 9 }
                                    Text { text: root.metric(modelData.heading_deg, 0, "°"); Layout.preferredWidth: 62; color: root.muted; font.pixelSize: 9 }
                                    Text { text: root.metric(modelData.battery_voltage_v, 2, " V"); Layout.preferredWidth: 65; color: root.muted; font.pixelSize: 9 }
                                    Text { text: root.metric(modelData.gps_speed_mps, 2, " m/s") + " / " + root.metric(modelData.gps_course_deg, 0, "°"); Layout.preferredWidth: 115; color: root.muted; font.pixelSize: 9 }
                                    Text { text: root.metric(modelData.mag_x, 0, "") + " / " + root.metric(modelData.mag_y, 0, "") + " / " + root.metric(modelData.mag_z, 0, ""); Layout.preferredWidth: 125; color: root.muted; font.pixelSize: 9; elide: Text.ElideRight }
                                    Item { Layout.fillWidth: true }
                                }
                            }
                        }
                    }
                }
            }
        }
        Text {
            Layout.fillWidth: true
            text: "Each row exposes logged motor outputs, acceleration, gyro, current, applied drive/PWM, heading, GPS response, and battery voltage. Save partial drafts while calibrating; firmware keeps the safe default mix until all 60 points are present and verified."
            color: root.muted; font.pixelSize: 9; wrapMode: Text.WordWrap
        }
    }
}
