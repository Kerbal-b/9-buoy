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


    function motorDrive(key) {
        const field = key === "rear" ? "rearMotorActualDrive" : key === "front_left" ? "frontLeftMotorActualDrive" : "frontRightMotorActualDrive"
        return Number(backendState[field] || 0)
    }

    function motorPwm(key) {
        const field = key === "rear" ? "rearMotorPwm" : key === "front_left" ? "frontLeftMotorPwm" : "frontRightMotorPwm"
        return backendState[field] === undefined ? -1 : Number(backendState[field])
    }

    component PanelButton: Button {
        id: panelButton
        property bool primary: false
        property bool danger: false
        implicitHeight: 34
        implicitWidth: Math.max(45, buttonLabel.implicitWidth + 20)
        background: Rectangle {
            radius: 8
            color: !panelButton.enabled ? "#172431" : panelButton.danger ? "#5a2b34" : panelButton.primary ? "#175662" : panelButton.hovered ? "#263c4d" : "#1c2d3b"
            border.color: !panelButton.enabled ? "#263643" : panelButton.danger ? "#a85b66" : panelButton.primary ? "#3c9ca7" : "#385064"
        }
        contentItem: Text {
            id: buttonLabel
            text: panelButton.text
            color: !panelButton.enabled ? "#657b8b" : panelButton.danger ? "#ffe3e6" : panelButton.primary ? "#e8ffff" : "#dbe8ef"
            font.pixelSize: 11
            font.bold: true
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
        }
    }

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 18
        spacing: 12

        RowLayout {
            Layout.fillWidth: true
            ColumnLayout { Layout.fillWidth: true; spacing: 2
                Text { text: "Propulsion"; color: "#eef5f9"; font.pixelSize: 24; font.bold: true }
                Text { text: "Motor output, bench controls, and live electrical load"; color: "#91a9b9"; font.pixelSize: 11 }
            }
            Rectangle {
                width: 86; height: 24; radius: 12
                color: root.isConnected() ? "#173b34" : "#3a3030"
                border.color: root.isConnected() ? "#346e58" : "#68504a"
                Text { anchors.centerIn: parent; text: root.isConnected() ? "CONNECTED" : "OFFLINE"; color: root.isConnected() ? "#78ddb2" : "#f2bb74"; font.pixelSize: 9; font.bold: true }
            }
            PanelButton { text: "STOP ALL MOTORS"; danger: true; enabled: root.isConnected(); onClicked: root.stopAllMotors() }
        }

        GridLayout {
            Layout.fillWidth: true
            columns: 3; columnSpacing: 12; rowSpacing: 0
            Repeater {
                model: [
                    { label: "BATTERY VOLTAGE", value: root.measuredMetric("BATTERY VOLTAGE"), accent: "#75baff" },
                    { label: "BATTERY CURRENT", value: root.measuredMetric("BATTERY CURRENT"), accent: "#f2bb74" },
                    { label: "BATTERY POWER", value: root.measuredMetric("BATTERY POWER"), accent: "#78ddb2" }
                ]
                delegate: Rectangle {
                    required property var modelData
                    Layout.fillWidth: true; Layout.preferredHeight: 88
                    radius: 14; color: "#111e2a"; border.color: "#2a4051"
                    ColumnLayout {
                        anchors.fill: parent; anchors.margins: 13; spacing: 4
                        Text { text: modelData.label; color: modelData.accent; font.pixelSize: 9; font.bold: true; font.letterSpacing: 1 }
                        Text { text: modelData.value; color: "#eef5f9"; font.pixelSize: 20; font.bold: true; elide: Text.ElideRight; Layout.fillWidth: true }
                    }
                }
            }
        }

        Rectangle {
            Layout.fillWidth: true; Layout.preferredHeight: 60
            radius: 12; color: "#132432"; border.color: "#2a4051"
            RowLayout {
                anchors.fill: parent; anchors.margins: 12; spacing: 10
                ColumnLayout { spacing: 2
                    Text { text: "BENCH TEST POWER"; color: "#91a9b9"; font.pixelSize: 9; font.bold: true; font.letterSpacing: 1 }
                    Text { text: "Hold a direction button to run; release to stop."; color: "#dbe8ef"; font.pixelSize: 11 }
                }
                Item { Layout.fillWidth: true }
                Slider {
                    id: testPowerSlider
                    Layout.preferredWidth: 175
                    from: 1; to: 100; stepSize: 1; value: root.testPowerPercent
                    enabled: !root.testRunning
                    onMoved: root.testPowerPercent = Math.round(value)
                    background: Rectangle {
                        x: testPowerSlider.leftPadding; y: testPowerSlider.topPadding + testPowerSlider.availableHeight / 2 - height / 2
                        width: testPowerSlider.availableWidth; height: 6; radius: 3; color: "#2a4051"
                        Rectangle { width: testPowerSlider.visualPosition * parent.width; height: parent.height; radius: 3; color: "#65d4da" }
                    }
                    handle: Rectangle { x: testPowerSlider.leftPadding + testPowerSlider.visualPosition * (testPowerSlider.availableWidth - width); y: testPowerSlider.topPadding + testPowerSlider.availableHeight / 2 - height / 2; width: 18; height: 18; radius: 9; color: "#eef5f9"; border.color: "#65d4da" }
                }
                Text { text: root.testPowerPercent + "%"; color: "#eef5f9"; font.pixelSize: 15; font.bold: true; Layout.preferredWidth: 44 }
                PanelButton { text: "Reload"; enabled: root.isConnected(); onClicked: root.reloadAllConfigurations() }
            }
        }

        GridLayout {
            id: motorGrid
            Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumHeight: 430
            columns: 3; columnSpacing: 12; rowSpacing: 0
            Repeater {
                model: root.motorList
                delegate: Rectangle {
                    id: motorCard
                    required property var modelData
                    Layout.fillWidth: true; Layout.fillHeight: true; Layout.minimumWidth: 245
                    radius: 16; color: "#111e2a"
                    border.width: root.activeMotorKey === modelData.key ? 2 : 1
                    border.color: root.activeMotorKey === modelData.key ? "#65d4da" : "#2a4051"

                    ColumnLayout {
                        anchors.fill: parent; anchors.margins: 14; spacing: 10
                        RowLayout {
                            Layout.fillWidth: true
                            Text { text: modelData.label; color: "#75baff"; font.pixelSize: 11; font.bold: true; font.letterSpacing: 1; Layout.fillWidth: true }
                            Text {
                                text: root.activeMotorKey === modelData.key ? "RUNNING" : root.dirtyProfiles[modelData.key] ? "EDITED" : root.profileFor(modelData.key).loaded ? "READY" : "NOT READ"
                                color: root.activeMotorKey === modelData.key ? "#78ddb2" : root.dirtyProfiles[modelData.key] ? "#f2bb74" : "#91a9b9"
                                font.pixelSize: 9; font.bold: true
                            }
                        }
                        RowLayout {
                            Layout.fillWidth: true
                            Text { text: root.motorPwm(modelData.key) < 0 ? "—" : root.motorDrive(modelData.key) + "%"; color: "#eef5f9"; font.pixelSize: 27; font.bold: true }
                            ColumnLayout { Layout.fillWidth: true; spacing: 1
                                Text { text: "APPLIED DRIVE"; color: "#91a9b9"; font.pixelSize: 9; font.bold: true }
                                Text { text: root.motorPwm(modelData.key) < 0 ? "PWM unavailable" : "PWM " + root.motorPwm(modelData.key) + " / 255"; color: "#91a9b9"; font.pixelSize: 10 }
                            }
                        }
                        Rectangle { Layout.fillWidth: true; height: 1; color: "#2a4051" }
                        RowLayout {
                            Layout.fillWidth: true
                            Text { text: "Motor orientation"; color: "#dbe8ef"; font.pixelSize: 11; Layout.fillWidth: true }
                            Text { text: root.profileFor(modelData.key).reversed ? "REVERSED" : "NORMAL"; color: root.profileFor(modelData.key).reversed ? "#f2bb74" : "#78ddb2"; font.pixelSize: 9; font.bold: true }
                            Switch {
                                id: orientationSwitch
                                checked: root.profileFor(modelData.key).reversed
                                enabled: root.profileFor(modelData.key).loaded && !root.testRunning
                                onToggled: root.setProfile(modelData.key, "reversed", checked)
                                indicator: Rectangle {
                                    implicitWidth: 44; implicitHeight: 22; x: orientationSwitch.leftPadding; y: parent.height / 2 - height / 2
                                    radius: 11; color: orientationSwitch.checked ? "#8b572d" : "#34495b"; border.color: orientationSwitch.checked ? "#f2bb74" : "#61798c"
                                    Rectangle { x: orientationSwitch.checked ? parent.width - width - 3 : 3; y: 3; width: 16; height: 16; radius: 8; color: "#f2f6f9" }
                                }
                            }
                        }
                        GridLayout {
                            Layout.fillWidth: true; columns: 2; columnSpacing: 8; rowSpacing: 7
                            Text { text: "Start PWM"; color: "#91a9b9"; font.pixelSize: 10 }
                            PwmValueInput { Layout.fillWidth: true; from: root.profileFor(modelData.key).minimum; to: root.profileFor(modelData.key).maximum; value: root.profileFor(modelData.key).startPwm; editable: true; enabled: root.profileFor(modelData.key).loaded && !root.testRunning; onValueModified: root.setProfile(modelData.key, "startPwm", value) }
                            Text { text: "Minimum PWM"; color: "#91a9b9"; font.pixelSize: 10 }
                            PwmValueInput { Layout.fillWidth: true; from: 0; to: root.profileFor(modelData.key).maximum; value: root.profileFor(modelData.key).minimum; editable: true; enabled: root.profileFor(modelData.key).loaded && !root.testRunning; onValueModified: root.setProfile(modelData.key, "minimum", value) }
                            Text { text: "Maximum PWM"; color: "#91a9b9"; font.pixelSize: 10 }
                            PwmValueInput { Layout.fillWidth: true; from: root.profileFor(modelData.key).minimum; to: 255; value: root.profileFor(modelData.key).maximum; editable: true; enabled: root.profileFor(modelData.key).loaded && !root.testRunning; onValueModified: root.setProfile(modelData.key, "maximum", value) }
                        }
                        Rectangle {
                            Layout.fillWidth: true; Layout.preferredHeight: 84
                            radius: 9; color: "#0c1823"; border.color: "#2a4051"
                            Column {
                                anchors.fill: parent; anchors.margins: 10; spacing: 5
                                Text { text: "PWM OPERATING BAND"; color: "#91a9b9"; font.pixelSize: 9; font.bold: true }
                                Item {
                                    width: parent.width; height: 15
                                    Rectangle { anchors.verticalCenter: parent.verticalCenter; width: parent.width; height: 5; radius: 3; color: "#2a4051" }
                                    Rectangle {
                                        anchors.verticalCenter: parent.verticalCenter; height: 5; radius: 3
                                        x: root.profileFor(modelData.key).minimum / 255 * parent.width
                                        width: Math.max(2, (root.profileFor(modelData.key).maximum - root.profileFor(modelData.key).minimum) / 255 * parent.width)
                                        color: "#236d76"
                                    }
                                    Rectangle {
                                        x: Math.max(0, Math.min(parent.width - width, root.motorPwm(modelData.key) / 255 * parent.width - width / 2))
                                        y: 0; width: 5; height: 15; radius: 2; color: "#65d4da"
                                        visible: root.motorPwm(modelData.key) >= 0
                                    }
                                }
                                RowLayout {
                                    width: parent.width
                                    Text { text: "Min " + root.profileFor(modelData.key).minimum; color: "#91a9b9"; font.pixelSize: 9; Layout.fillWidth: true }
                                    Text { text: "Curve " + Number(root.profileFor(modelData.key).curve).toFixed(1) + " · Ramp " + Number(root.profileFor(modelData.key).rampSeconds).toFixed(1) + "s"; color: "#91a9b9"; font.pixelSize: 9 }
                                }
                            }
                        }
                        Item { Layout.fillHeight: true }
                        RowLayout {
                            Layout.fillWidth: true; spacing: 7
                            PanelButton {
                                Layout.fillWidth: true; text: "REVERSE"; danger: true
                                enabled: root.isConnected() && root.profileFor(modelData.key).loaded && !root.dirtyProfiles[modelData.key] && (!root.activeMotorKey || (root.activeMotorKey === modelData.key && root.activeDirection === "reverse"))
                                onPressed: root.beginMotorTest(modelData.key, -1)
                                onReleased: root.endMotorTest(modelData.key, "released")
                                onCanceled: root.endMotorTest(modelData.key, "canceled")
                            }
                            PanelButton {
                                Layout.fillWidth: true; text: "FORWARD"; primary: true
                                enabled: root.isConnected() && root.profileFor(modelData.key).loaded && !root.dirtyProfiles[modelData.key] && (!root.activeMotorKey || (root.activeMotorKey === modelData.key && root.activeDirection === "forward"))
                                onPressed: root.beginMotorTest(modelData.key, 1)
                                onReleased: root.endMotorTest(modelData.key, "released")
                                onCanceled: root.endMotorTest(modelData.key, "canceled")
                            }
                        }
                        Rectangle { Layout.fillWidth: true; height: 1; color: "#2a4051" }
                        RowLayout {
                            Layout.fillWidth: true; spacing: 8
                            PanelButton { text: "Apply to buoy"; enabled: root.isConnected() && root.profileFor(modelData.key).loaded && !root.testRunning; onClicked: root.saveProfile(modelData.key) }
                            Text { text: root.dirtyProfiles[modelData.key] ? "Changes pending" : root.profileFor(modelData.key).loaded ? "RAM only" : "Read settings"; color: root.dirtyProfiles[modelData.key] ? "#f2bb74" : "#91a9b9"; font.pixelSize: 10; Layout.fillWidth: true; horizontalAlignment: Text.AlignRight }
                        }
                    }
                }
            }
        }

        Text { Layout.fillWidth: true; text: "Motor settings are currently applied in buoy RAM and reset on reboot. An SD profile and GPS/heading based water calibration are planned."; color: "#91a9b9"; font.pixelSize: 10; wrapMode: Text.WordWrap }
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
