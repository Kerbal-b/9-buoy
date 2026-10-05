import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Item {
    id: root

    property var backendObject
    property var backendState: ({})
    property int testPowerPercent: 15
    property string activeMotorKey: ""
    property string activeDirection: ""
    property bool testRunning: activeMotorKey !== ""
    property bool configWasConnected: false
    property int configFetchIdSeen: 0
    property int configVisitId: 0
    property string lastConfigRequestKey: ""

    property var motorList: [
        { key: "front_left", label: "LEFT MOTOR", name: "Front left" },
        { key: "rear", label: "REAR MOTOR", name: "Rear" },
        { key: "front_right", label: "RIGHT MOTOR", name: "Front right" }
    ]
    property var motorProfiles: ({
        front_left: { startPwm: 171, pulseMs: 260, minimum: 130, maximum: 255, curve: 2.0, rampSeconds: 1.0, reversed: true, loaded: false },
        rear: { startPwm: 171, pulseMs: 260, minimum: 130, maximum: 255, curve: 2.0, rampSeconds: 1.0, reversed: true, loaded: false },
        front_right: { startPwm: 171, pulseMs: 260, minimum: 130, maximum: 255, curve: 2.0, rampSeconds: 1.0, reversed: false, loaded: false }
    })
    property var dirtyProfiles: ({})

    function isConnected() {
        return String(backendState.serialStatus || "").startsWith("Connected")
    }

    function profileFor(key) {
        return motorProfiles[key] || motorProfiles.rear
    }

    function setProfile(key, field, value) {
        const profiles = Object.assign({}, motorProfiles)
        const profile = Object.assign({}, profileFor(key))
        profile[field] = value
        if (field === "minimum") {
            profile.maximum = Math.max(profile.maximum, value)
            profile.startPwm = Math.max(profile.startPwm, value)
        } else if (field === "maximum") {
            profile.minimum = Math.min(profile.minimum, value)
            profile.startPwm = Math.min(profile.startPwm, value)
        } else if (field === "startPwm") {
            profile.startPwm = Math.max(profile.minimum, Math.min(profile.maximum, value))
        }
        profiles[key] = profile
        motorProfiles = profiles
        const dirty = Object.assign({}, dirtyProfiles)
        dirty[key] = true
        dirtyProfiles = dirty
    }

    function requestAllConfigurations(force) {
        if (!backendObject || !isConnected()) return
        const requestKey = String(configVisitId) + "|" + String(backendState.serialTarget || "")
        if (!force && requestKey === lastConfigRequestKey) return
        lastConfigRequestKey = requestKey
        for (let index = 0; index < motorList.length; ++index) {
            backendObject.requestMotorConfiguration(motorList[index].key, "forward")
        }
    }

    function activate() {
        configVisitId += 1
        requestAllConfigurations(true)
    }

    function deactivate() {
        if (testRunning) stopAllMotors()
    }

    function syncConfiguration() {
        if (!backendObject) return
        const state = backendObject.state || backendState
        const fetchId = Number(state.motorConfigFetchId || 0)
        if (!Number.isFinite(fetchId) || fetchId <= configFetchIdSeen) return
        configFetchIdSeen = fetchId

        const key = String(state.motorConfigMotor || "").toLowerCase()
        if (String(state.motorConfigDirection || "").toLowerCase() !== "forward" || !motorProfiles[key]) return
        if (dirtyProfiles[key]) return

        const profiles = Object.assign({}, motorProfiles)
        profiles[key] = {
            startPwm: Number(state.motorConfigStartBoostPwm),
            pulseMs: Number(state.motorConfigStartBoostMs),
            minimum: Number(state.motorConfigSustainMinPwm),
            maximum: Number(state.motorConfigMaxPwm),
            curve: Number(state.motorConfigCurveTimes100) / 100.0,
            rampSeconds: Number(state.motorConfigRampTenths) / 10.0,
            reversed: Boolean(state.motorConfigDefaultReversed),
            loaded: true
        }
        motorProfiles = profiles
    }

    function saveProfile(key) {
        if (!backendObject || !isConnected()) return
        const profile = profileFor(key)
        backendObject.sendSharedMotorCalibration(
            key,
            profile.startPwm,
            profile.pulseMs,
            profile.minimum,
            profile.maximum,
            profile.curve,
            profile.rampSeconds,
            profile.reversed
        )
        const dirty = Object.assign({}, dirtyProfiles)
        dirty[key] = false
        dirtyProfiles = dirty
        const profiles = Object.assign({}, motorProfiles)
        profiles[key] = Object.assign({}, profile, { loaded: true })
        motorProfiles = profiles
    }

    function reloadAllConfigurations() {
        const dirty = Object.assign({}, dirtyProfiles)
        const profiles = Object.assign({}, motorProfiles)
        for (let index = 0; index < motorList.length; ++index) {
            const key = motorList[index].key
            dirty[key] = false
            profiles[key] = Object.assign({}, profiles[key], { loaded: false })
        }
        dirtyProfiles = dirty
        motorProfiles = profiles
        configFetchIdSeen = Number((backendObject && backendObject.state.motorConfigFetchId) || 0)
        lastConfigRequestKey = ""
        requestAllConfigurations(true)
    }

    function beginMotorTest(key, direction) {
        if (!backendObject || !isConnected()) return
        if (activeMotorKey && activeMotorKey !== key) return
        activeMotorKey = key
        activeDirection = direction > 0 ? "forward" : "reverse"
        backendObject.sendMotorTestPower(key, direction * testPowerPercent)
    }

    function endMotorTest(key, reason) {
        if (!activeMotorKey || (key && activeMotorKey !== key)) return
        if (backendObject && isConnected()) backendObject.stopMotorTest(reason || "unknown")
        activeMotorKey = ""
        activeDirection = ""
    }

    function stopAllMotors() {
        if (backendObject) backendObject.stopAllMotors()
        activeMotorKey = ""
        activeDirection = ""
    }

    function motorTelemetry(key) {
        const fields = key === "rear"
            ? ["rearMotorActualDrive", "rearMotorPwm"]
            : key === "front_left"
                ? ["frontLeftMotorActualDrive", "frontLeftMotorPwm"]
                : ["frontRightMotorActualDrive", "frontRightMotorPwm"]
        const drive = Number(backendState[fields[0]] || 0)
        const pwm = Number(backendState[fields[1]] === undefined ? -1 : backendState[fields[1]])
        return pwm >= 0 ? (drive + "% drive | PWM " + pwm + "/255") : "Waiting for motor output telemetry"
    }

    function batteryVoltage() {
        const match = String(backendState.batteryStatus || "").match(/(-?\d+(?:\.\d+)?)\s*V/i)
        return match ? Number(match[1]) : NaN
    }

    function batteryCurrent() {
        const match = String(backendState.currentDraw || "").match(/(-?\d+(?:\.\d+)?)\s*A/i)
        return match ? Math.abs(Number(match[1])) : NaN
    }

    function batteryPower() {
        const voltage = batteryVoltage()
        const current = batteryCurrent()
        return Number.isFinite(voltage) && Number.isFinite(current) ? voltage * current : NaN
    }

    function measuredValue(value, unit, digits) {
        return Number.isFinite(value) ? value.toFixed(digits) + " " + unit : "Waiting for telemetry"
    }

    function measuredMetric(label) {
        if (label === "BATTERY VOLTAGE") return measuredValue(batteryVoltage(), "V", 2)
        if (label === "BATTERY CURRENT") return measuredValue(batteryCurrent(), "A", 2)
        return measuredValue(batteryPower(), "W", 1)
    }

    Rectangle {
        anchors.fill: parent
        radius: 12
        color: "#0b1119"
        border.color: "#395166"

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 16
            spacing: 12

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 208
                radius: 10
                color: "#111c28"
                border.color: "#395166"
                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 12
                    spacing: 14
                    Image {
                        Layout.preferredWidth: 220
                        Layout.fillHeight: true
                        source: "../assets/underwater-thruster.png"
                        sourceSize.width: 500
                        fillMode: Image.PreserveAspectFit
                        smooth: true
                        mipmap: true
                    }
                    ColumnLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        spacing: 6
                        RowLayout {
                            Layout.fillWidth: true
                            Text { text: "Propulsion overview"; color: "#e6ecf2"; font.pixelSize: 20; font.bold: true }
                            Text { Layout.fillWidth: true; text: root.isConnected() ? "Buoy connected" : "Buoy not connected"; color: root.isConnected() ? "#55d6a2" : "#9eb5c7"; font.pixelSize: 11; horizontalAlignment: Text.AlignRight }
                            Button {
                                text: "STOP ALL MOTORS"
                                enabled: root.isConnected()
                                onClicked: root.stopAllMotors()
                                background: Rectangle { radius: 7; color: parent.down ? "#752e37" : "#9d3845"; border.color: "#e47b84" }
                                contentItem: Text { text: parent.text; color: "white"; font.pixelSize: 11; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                            }
                        }
                        RowLayout {
                            Layout.fillWidth: true
                            Text { text: "BENCH TEST POWER"; color: "#8ca3b5"; font.pixelSize: 10; font.bold: true }
                            Slider {
                                id: testPowerSlider
                                Layout.preferredWidth: 190
                                from: 1; to: 100; stepSize: 1; value: root.testPowerPercent
                                enabled: !root.testRunning
                                onMoved: root.testPowerPercent = Math.round(value)
                                background: Rectangle { x: testPowerSlider.leftPadding; y: testPowerSlider.topPadding + testPowerSlider.availableHeight / 2 - height / 2; width: testPowerSlider.availableWidth; height: 6; radius: 3; color: "#273746" }
                                handle: Rectangle { x: testPowerSlider.leftPadding + testPowerSlider.visualPosition * (testPowerSlider.availableWidth - width); y: testPowerSlider.topPadding + testPowerSlider.availableHeight / 2 - height / 2; width: 18; height: 18; radius: 9; color: "#e6ecf2"; border.color: "#5aa8dc" }
                            }
                            Text { text: root.testPowerPercent + "%"; color: "#e6ecf2"; font.pixelSize: 12; font.bold: true }
                            Text { Layout.fillWidth: true; text: "Hold a direction button to test; release to stop."; color: "#829aad"; font.pixelSize: 10; elide: Text.ElideRight }
                            Button {
                                text: "Reload settings"
                                enabled: root.isConnected()
                                onClicked: root.reloadAllConfigurations()
                                background: Rectangle { radius: 6; color: parent.down ? "#244a63" : "#1b3448"; border.color: "#55748d" }
                                contentItem: Text { text: parent.text; color: "#e6ecf2"; font.pixelSize: 10; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                            }
                        }
                        RowLayout {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            spacing: 8
                            Repeater {
                                model: ["BATTERY VOLTAGE", "BATTERY CURRENT", "BATTERY INPUT POWER"]
                                delegate: Rectangle {
                                    required property string modelData
                                    Layout.fillWidth: true; Layout.fillHeight: true
                                    radius: 6; color: "#0b131d"; border.color: "#263949"
                                    ColumnLayout {
                                        anchors.fill: parent; anchors.margins: 8; spacing: 3
                                        Text { text: modelData; color: "#7890a3"; font.pixelSize: 9; font.bold: true }
                                        Text { text: root.measuredMetric(modelData); color: "#e6ecf2"; font.pixelSize: 14; font.bold: true; Layout.fillWidth: true; elide: Text.ElideRight }
                                        Text { text: String(root.backendState.telemetrySource || "No telemetry source"); color: "#829aad"; font.pixelSize: 9; elide: Text.ElideRight }
                                    }
                                }
                            }
                        }
                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 8
                            Text { text: "MOTOR CURRENT CALIBRATION"; color: "#7890a3"; font.pixelSize: 9; font.bold: true }
                            Repeater {
                                model: ["30%", "50%", "100%"]
                                delegate: Rectangle {
                                    required property string modelData
                                    Layout.fillWidth: true; Layout.preferredHeight: 26
                                    radius: 5; color: "#17232e"; border.color: "#30465a"
                                    RowLayout {
                                        anchors.fill: parent; anchors.leftMargin: 7; anchors.rightMargin: 7
                                        Text { text: modelData; color: "#c8d5df"; font.pixelSize: 10; font.bold: true }
                                        Text { Layout.fillWidth: true; text: "No sample"; color: "#829aad"; font.pixelSize: 9; horizontalAlignment: Text.AlignRight }
                                    }
                                }
                            }
                            Text {
                                Layout.fillWidth: true
                                text: "Mission propulsion estimate: awaiting route time and motor current calibration."
                                color: "#efb76f"; font.pixelSize: 9; elide: Text.ElideRight
                            }
                        }
                        Text {
                            Layout.fillWidth: true
                            text: "Starting model: Pprop = Pidle + sum(P100_i * d_i^3); d_i is normalized PWM from min/max/curve. Mission Wh = integral(Pprop dt)/3600. Fit to measured current/speed and route times."
                            color: "#829aad"; font.pixelSize: 9; wrapMode: Text.WordWrap; maximumLineCount: 2; elide: Text.ElideRight
                        }
                    }
                }
            }

            GridLayout {
                id: motorGrid
                Layout.fillWidth: true
                Layout.fillHeight: true
                columns: 3
                columnSpacing: 12
                rowSpacing: 0
                Repeater {
                    model: root.motorList
                    delegate: Rectangle {
                        id: motorCard
                        required property var modelData
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        Layout.minimumWidth: 245
                        radius: 11
                        color: "#111c28"
                        border.width: root.activeMotorKey === modelData.key ? 2 : 1
                        border.color: root.activeMotorKey === modelData.key ? "#55d6a2" : "#395166"

                        ColumnLayout {
                            anchors.fill: parent
                            anchors.margins: 13
                            spacing: 8

                            RowLayout {
                                Layout.fillWidth: true
                                Text { text: modelData.label; color: "#80bfff"; font.pixelSize: 14; font.bold: true }
                                Item { Layout.fillWidth: true }
                                Text {
                                    text: root.activeMotorKey === modelData.key
                                        ? "TESTING " + root.activeDirection.toUpperCase()
                                        : (root.dirtyProfiles[modelData.key] ? "UNSAVED" : (root.profileFor(modelData.key).loaded ? "LOADED" : "NOT READ"))
                                    color: root.activeMotorKey === modelData.key ? "#55d6a2" : (root.dirtyProfiles[modelData.key] ? "#efb76f" : "#829aad")
                                    font.pixelSize: 9; font.bold: true
                                }
                            }

                            RowLayout {
                                Layout.fillWidth: true
                                Text { text: "MOTOR ORIENTATION"; color: "#aebdca"; font.pixelSize: 10; font.bold: true; Layout.fillWidth: true }
                                Text { text: root.profileFor(modelData.key).reversed ? "REVERSED" : "NORMAL"; color: root.profileFor(modelData.key).reversed ? "#efb76f" : "#79d3a6"; font.pixelSize: 10; font.bold: true }
                                Switch {
                                    id: orientationSwitch
                                    checked: root.profileFor(modelData.key).reversed
                                    enabled: root.profileFor(modelData.key).loaded && !root.testRunning
                                    onToggled: root.setProfile(modelData.key, "reversed", checked)
                                    indicator: Rectangle {
                                        implicitWidth: 44; implicitHeight: 22; x: orientationSwitch.leftPadding; y: parent.height / 2 - height / 2
                                        radius: 11; color: orientationSwitch.checked ? "#8b572d" : "#34495b"; border.color: orientationSwitch.checked ? "#efb76f" : "#61798c"
                                        Rectangle { x: orientationSwitch.checked ? parent.width - width - 3 : 3; y: 3; width: 16; height: 16; radius: 8; color: "#f2f6f9" }
                                    }
                                }
                            }

                            Rectangle { Layout.fillWidth: true; height: 1; color: "#263949" }

                            GridLayout {
                                Layout.fillWidth: true
                                columns: 2
                                columnSpacing: 9
                                rowSpacing: 5
                                Text { text: "Start PWM"; color: "#aebdca"; font.pixelSize: 11 }
                                PwmValueInput {
                                    Layout.fillWidth: true; from: root.profileFor(modelData.key).minimum; to: root.profileFor(modelData.key).maximum
                                    value: root.profileFor(modelData.key).startPwm; editable: true; enabled: root.profileFor(modelData.key).loaded && !root.testRunning
                                    onValueModified: root.setProfile(modelData.key, "startPwm", value)
                                }
                                Text { text: "Minimum PWM"; color: "#aebdca"; font.pixelSize: 11 }
                                PwmValueInput {
                                    Layout.fillWidth: true; from: 0; to: root.profileFor(modelData.key).maximum
                                    value: root.profileFor(modelData.key).minimum; editable: true; enabled: root.profileFor(modelData.key).loaded && !root.testRunning
                                    onValueModified: root.setProfile(modelData.key, "minimum", value)
                                }
                                Text { text: "Maximum PWM"; color: "#aebdca"; font.pixelSize: 11 }
                                PwmValueInput {
                                    Layout.fillWidth: true; from: root.profileFor(modelData.key).minimum; to: 255
                                    value: root.profileFor(modelData.key).maximum; editable: true; enabled: root.profileFor(modelData.key).loaded && !root.testRunning
                                    onValueModified: root.setProfile(modelData.key, "maximum", value)
                                }
                            }

                            RowLayout {
                                Layout.fillWidth: true
                                Layout.topMargin: 2
                                Button {
                                    Layout.fillWidth: true
                                    text: "REVERSE"
                                    enabled: root.isConnected() && root.profileFor(modelData.key).loaded && !root.dirtyProfiles[modelData.key] && (!root.activeMotorKey || (root.activeMotorKey === modelData.key && root.activeDirection === "reverse"))
                                    onPressed: root.beginMotorTest(modelData.key, -1)
                                    onReleased: root.endMotorTest(modelData.key, "released")
                                    onCanceled: root.endMotorTest(modelData.key, "canceled")
                                    background: Rectangle { radius: 6; color: parent.down ? "#74363b" : "#552a31"; border.color: "#b65962" }
                                    contentItem: Text { text: parent.text; color: "#ffe9eb"; font.pixelSize: 11; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                                }
                                Button {
                                    Layout.fillWidth: true
                                    text: "FORWARD"
                                    enabled: root.isConnected() && root.profileFor(modelData.key).loaded && !root.dirtyProfiles[modelData.key] && (!root.activeMotorKey || (root.activeMotorKey === modelData.key && root.activeDirection === "forward"))
                                    onPressed: root.beginMotorTest(modelData.key, 1)
                                    onReleased: root.endMotorTest(modelData.key, "released")
                                    onCanceled: root.endMotorTest(modelData.key, "canceled")
                                    background: Rectangle { radius: 6; color: parent.down ? "#23604d" : "#174536"; border.color: "#3e9473" }
                                    contentItem: Text { text: parent.text; color: "#e4fff3"; font.pixelSize: 11; font.bold: true; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                                }
                            }

                            Rectangle {
                                Layout.fillWidth: true
                                Layout.preferredHeight: 42
                                radius: 6
                                color: "#0b131d"
                                border.color: "#263949"
                                ColumnLayout {
                                    anchors.fill: parent; anchors.leftMargin: 9; anchors.rightMargin: 9; anchors.topMargin: 4; anchors.bottomMargin: 4; spacing: 1
                                    Text { text: "APPLIED OUTPUT"; color: "#7890a3"; font.pixelSize: 9; font.bold: true }
                                    Text { text: root.motorTelemetry(modelData.key); color: "#d5e3ec"; font.pixelSize: 11; elide: Text.ElideRight; Layout.fillWidth: true }
                                }
                            }

                            RowLayout {
                                Layout.fillWidth: true
                                Button {
                                    text: "Apply to buoy"
                                    enabled: root.isConnected() && root.profileFor(modelData.key).loaded && !root.testRunning
                                    onClicked: root.saveProfile(modelData.key)
                                    background: Rectangle { radius: 6; color: parent.enabled ? (parent.down ? "#244a63" : "#1b3448") : "#17232e"; border.color: parent.enabled ? "#55748d" : "#2d3e4b" }
                                    contentItem: Text { text: parent.text; color: parent.enabled ? "#e6ecf2" : "#71889b"; font.pixelSize: 11; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                                }
                                Text {
                                    Layout.fillWidth: true
                                    text: root.dirtyProfiles[modelData.key] ? "Apply changes before testing" : (root.profileFor(modelData.key).loaded ? "Saved settings" : "Read settings from buoy")
                                    color: root.dirtyProfiles[modelData.key] ? "#efb76f" : "#829aad"; font.pixelSize: 9; elide: Text.ElideRight
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    Connections {
        target: root.backendObject || null
        function onStateChanged() {
            const connected = root.isConnected()
            const wasConnected = root.configWasConnected
            root.configWasConnected = connected
            if (connected && !wasConnected && String(root.backendObject.state.dashboardTab) === "motors") {
                root.configFetchIdSeen = Number((root.backendObject.state.motorConfigFetchId) || 0)
                root.lastConfigRequestKey = ""
                root.requestAllConfigurations(false)
            }
            if (!connected && wasConnected) root.endMotorTest("")
            root.syncConfiguration()
        }
    }
}
