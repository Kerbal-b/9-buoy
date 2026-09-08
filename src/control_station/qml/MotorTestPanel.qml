import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Item {
    id: root

    property var backendObject
    property var backendState: ({})

    property int selectedMotorIndex: 0
    property int testPowerPercent: 0
    property bool testRunning: false

    property int calibrationStartBoostPwm: 96
    property int calibrationStartBoostMs: 250
    property int calibrationSustainMinPwm: 72
    property int calibrationMaxPwm: 255
    property int calibrationCurveTimes100: 125
    property int calibrationRampTenths: 20
    property bool calibrationDefaultReversed: false

    property int configFetchIdSeen: 0
    property int configAutoLoadVisitId: 0
    property int configConnectionGeneration: 0
    property bool configWasConnected: false
    property string configLastRequestedKey: ""

    function clamp(value, minimum, maximum) {
        return Math.max(minimum, Math.min(maximum, value))
    }

    function motorKey(index) {
        if (index === 1) return "front_left"
        if (index === 2) return "front_right"
        return "rear"
    }

    function motorLabel(index) {
        if (index === 1) return "Front Left"
        if (index === 2) return "Front Right"
        return "Rear"
    }

    function isConnected() {
        return String(backendState.serialStatus || "").indexOf("Connected") === 0
    }

    function numericPrefix(text) {
        if (text === null || text === undefined) return NaN
        const matches = String(text).match(/-?\d+(?:\.\d+)?/)
        return matches ? Number(matches[0]) : NaN
    }

    function formatValue(value, digits) {
        return Number.isFinite(value) ? value.toFixed(digits) : "N/A"
    }

    function normalizePwmValues() {
        calibrationSustainMinPwm = clamp(Math.round(calibrationSustainMinPwm), 0, 255)
        calibrationMaxPwm = clamp(Math.round(calibrationMaxPwm), calibrationSustainMinPwm, 255)
        calibrationStartBoostPwm = clamp(
            Math.round(calibrationStartBoostPwm),
            calibrationSustainMinPwm,
            calibrationMaxPwm
        )
    }

    function steadyPwmForPercent(percent) {
        const magnitude = clamp(Math.abs(Number(percent) || 0), 0, 100)
        if (magnitude <= 0) return 0
        const drive = 11 + ((255 - 11) * magnitude / 100.0)
        const normalizedDrive = drive / 255.0
        const curve = Math.max(0.1, calibrationCurveTimes100 / 100.0)
        return calibrationSustainMinPwm +
            ((calibrationMaxPwm - calibrationSustainMinPwm) * Math.pow(normalizedDrive, curve))
    }

    function currentPwmValue() {
        return Math.round(steadyPwmForPercent(testPowerPercent))
    }

    function currentDutyPercent() {
        return currentPwmValue() * 100.0 / 255.0
    }

    function supplyVoltage() {
        return numericPrefix(backendState.batteryStatus)
    }

    function estimatedAverageMotorVoltage() {
        const voltage = supplyVoltage()
        if (!Number.isFinite(voltage)) return NaN
        const magnitude = voltage * currentPwmValue() / 255.0
        return testPowerPercent < 0 ? -magnitude : magnitude
    }

    function configRequestKey() {
        return motorKey(selectedMotorIndex) + "|" +
            configAutoLoadVisitId + "|" + configConnectionGeneration
    }

    function requestConfiguration(force) {
        if (!isConnected() || !backendObject) return

        const requestKey = configRequestKey()
        if (!force && requestKey === configLastRequestedKey) return

        configLastRequestedKey = requestKey
        backendObject.requestMotorConfiguration(motorKey(selectedMotorIndex), "forward")
    }

    function syncConfiguration() {
        if (String(backendState.motorConfigMotor || "").toLowerCase() !== motorKey(selectedMotorIndex)) return
        if (String(backendState.motorConfigDirection || "").toLowerCase() !== "forward") return

        const fetchId = Number(backendState.motorConfigFetchId || 0)
        if (!Number.isFinite(fetchId) || fetchId <= configFetchIdSeen) return

        configFetchIdSeen = fetchId
        calibrationStartBoostPwm = Number(backendState.motorConfigStartBoostPwm || 0)
        calibrationStartBoostMs = Number(backendState.motorConfigStartBoostMs || 0)
        calibrationSustainMinPwm = Number(backendState.motorConfigSustainMinPwm || 0)
        calibrationMaxPwm = Number(backendState.motorConfigMaxPwm || 0)
        calibrationCurveTimes100 = Number(backendState.motorConfigCurveTimes100 || calibrationCurveTimes100)
        calibrationRampTenths = Number(backendState.motorConfigRampTenths || calibrationRampTenths)
        calibrationDefaultReversed = Boolean(backendState.motorConfigDefaultReversed)
        normalizePwmValues()
    }

    function activate() {
        configAutoLoadVisitId += 1
        requestConfiguration(true)
    }

    function selectMotor(index) {
        if (selectedMotorIndex === index) return
        stopTest()
        selectedMotorIndex = index
        configFetchIdSeen = 0
        requestConfiguration(false)
    }

    function movePower(value) {
        const nextPower = clamp(Math.round(value), -100, 100)
        const previousPower = testPowerPercent
        testPowerPercent = nextPower

        if (!backendObject || !isConnected()) {
            testRunning = false
            return
        }

        if (nextPower === 0) {
            backendObject.stopMotorTest()
            testRunning = false
            return
        }

        if (testRunning && ((previousPower < 0 && nextPower > 0) || (previousPower > 0 && nextPower < 0))) {
            backendObject.stopMotorTest()
        }

        backendObject.sendMotorTestPower(motorKey(selectedMotorIndex), nextPower)
        testRunning = true
    }

    function stopTest() {
        const shouldSendStop = testRunning || testPowerPercent !== 0
        testPowerPercent = 0
        powerSlider.value = 0
        testRunning = false
        if (shouldSendStop && backendObject && isConnected()) backendObject.stopMotorTest()
    }

    function deactivate() {
        testPowerPercent = 0
        powerSlider.value = 0
        testRunning = false
    }

    function applyConfiguration() {
        if (!backendObject || !isConnected()) return
        backendObject.sendSharedMotorCalibration(
            motorKey(selectedMotorIndex),
            calibrationStartBoostPwm,
            calibrationStartBoostMs,
            calibrationSustainMinPwm,
            calibrationMaxPwm,
            calibrationCurveTimes100 / 100.0,
            calibrationRampTenths / 10.0,
            calibrationDefaultReversed
        )
    }

    ScrollView {
        id: motorScroll
        anchors.fill: parent
        anchors.margins: 1
        clip: true

        Column {
            x: 18
            y: 16
            width: motorScroll.availableWidth - 36
            spacing: 14

            Text {
                text: "Motor Test"
                color: "#e6ecf2"
                font.pixelSize: 22
                font.bold: true
            }

            Text {
                width: parent.width
                wrapMode: Text.WordWrap
                text: "Select a motor, then move the throttle away from center to begin. Left runs in reverse; right runs forward. Return to center or press Stop Motor to stop."
                color: "#9eb5c7"
                font.pixelSize: 13
            }

            Rectangle {
                width: parent.width
                height: controlColumn.implicitHeight + 32
                radius: 12
                color: "#121c28"
                border.color: testRunning ? "#4fb286" : "#33485d"
                border.width: 1

                Column {
                    id: controlColumn
                    x: 16
                    y: 16
                    width: parent.width - 32
                    spacing: 12

                    Row {
                        width: parent.width
                        spacing: 10

                        Text {
                            text: "MOTOR"
                            color: "#7f98ad"
                            font.pixelSize: 12
                            font.bold: true
                            anchors.verticalCenter: parent.verticalCenter
                        }

                        Item { width: 10; height: 1 }

                        ButtonGroup { id: motorButtonGroup }

                        Button {
                            id: rearMotorButton
                            width: (parent.width - 92) / 3
                            height: 38
                            text: "Rear"
                            checkable: true
                            checked: selectedMotorIndex === 0
                            ButtonGroup.group: motorButtonGroup
                            onClicked: selectMotor(0)

                            contentItem: Text {
                                text: rearMotorButton.text
                                color: rearMotorButton.checked ? "#ffffff" : "#bdcad5"
                                font.pixelSize: 13
                                font.bold: rearMotorButton.checked
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                            }

                            background: Rectangle {
                                radius: 7
                                color: rearMotorButton.checked
                                    ? "#167a68"
                                    : (rearMotorButton.hovered ? "#26394a" : "#1a2733")
                                border.color: rearMotorButton.checked ? "#62e6bd" : "#40566a"
                                border.width: rearMotorButton.checked ? 2 : 1
                            }
                        }

                        Button {
                            id: frontLeftMotorButton
                            width: (parent.width - 92) / 3
                            height: 38
                            text: "Front Left"
                            checkable: true
                            checked: selectedMotorIndex === 1
                            ButtonGroup.group: motorButtonGroup
                            onClicked: selectMotor(1)

                            contentItem: Text {
                                text: frontLeftMotorButton.text
                                color: frontLeftMotorButton.checked ? "#ffffff" : "#bdcad5"
                                font.pixelSize: 13
                                font.bold: frontLeftMotorButton.checked
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                            }

                            background: Rectangle {
                                radius: 7
                                color: frontLeftMotorButton.checked
                                    ? "#167a68"
                                    : (frontLeftMotorButton.hovered ? "#26394a" : "#1a2733")
                                border.color: frontLeftMotorButton.checked ? "#62e6bd" : "#40566a"
                                border.width: frontLeftMotorButton.checked ? 2 : 1
                            }
                        }

                        Button {
                            id: frontRightMotorButton
                            width: (parent.width - 92) / 3
                            height: 38
                            text: "Front Right"
                            checkable: true
                            checked: selectedMotorIndex === 2
                            ButtonGroup.group: motorButtonGroup
                            onClicked: selectMotor(2)

                            contentItem: Text {
                                text: frontRightMotorButton.text
                                color: frontRightMotorButton.checked ? "#ffffff" : "#bdcad5"
                                font.pixelSize: 13
                                font.bold: frontRightMotorButton.checked
                                horizontalAlignment: Text.AlignHCenter
                                verticalAlignment: Text.AlignVCenter
                            }

                            background: Rectangle {
                                radius: 7
                                color: frontRightMotorButton.checked
                                    ? "#167a68"
                                    : (frontRightMotorButton.hovered ? "#26394a" : "#1a2733")
                                border.color: frontRightMotorButton.checked ? "#62e6bd" : "#40566a"
                                border.width: frontRightMotorButton.checked ? 2 : 1
                            }
                        }
                    }

                    Row {
                        width: parent.width

                        Text {
                            width: parent.width / 3
                            text: "REVERSE"
                            color: "#e07a7a"
                            font.pixelSize: 12
                            font.bold: true
                            horizontalAlignment: Text.AlignLeft
                        }

                        Text {
                            width: parent.width / 3
                            text: testPowerPercent === 0 ? "STOPPED" : (Math.abs(testPowerPercent) + "%")
                            color: testRunning ? "#79d3a6" : "#d5dde5"
                            font.pixelSize: 15
                            font.bold: true
                            horizontalAlignment: Text.AlignHCenter
                        }

                        Text {
                            width: parent.width / 3
                            text: "FORWARD"
                            color: "#6eb6ff"
                            font.pixelSize: 12
                            font.bold: true
                            horizontalAlignment: Text.AlignRight
                        }
                    }

                    Slider {
                        id: powerSlider
                        width: parent.width
                        from: -100
                        to: 100
                        stepSize: 1
                        snapMode: Slider.SnapAlways
                        value: testPowerPercent
                        enabled: isConnected()
                        onMoved: movePower(value)

                        background: Rectangle {
                            x: powerSlider.leftPadding
                            y: powerSlider.topPadding + powerSlider.availableHeight / 2 - height / 2
                            width: powerSlider.availableWidth
                            height: 8
                            radius: 4
                            color: "#263747"

                            Rectangle {
                                x: parent.width / 2 - 1
                                y: -5
                                width: 2
                                height: 18
                                radius: 1
                                color: "#dce5ed"
                            }
                        }
                    }

                    Row {
                        width: parent.width

                        Text {
                            width: parent.width / 3
                            text: "-100%"
                            color: "#8094a5"
                            font.pixelSize: 11
                        }

                        Text {
                            width: parent.width / 3
                            text: "0"
                            color: "#b9c6d1"
                            font.pixelSize: 11
                            horizontalAlignment: Text.AlignHCenter
                        }

                        Text {
                            width: parent.width / 3
                            text: "+100%"
                            color: "#8094a5"
                            font.pixelSize: 11
                            horizontalAlignment: Text.AlignRight
                        }
                    }

                    Row {
                        width: parent.width
                        spacing: 12

                        Button {
                            width: 150
                            text: "Stop Motor"
                            enabled: isConnected()
                            onClicked: stopTest()
                        }

                        Text {
                            width: parent.width - 162
                            text: isConnected()
                                ? (testRunning ? "Running " + motorLabel(selectedMotorIndex) : "Ready — " + motorLabel(selectedMotorIndex))
                                : "Connect to the buoy to test a motor"
                            color: testRunning ? "#79d3a6" : "#a9bac8"
                            font.pixelSize: 13
                            verticalAlignment: Text.AlignVCenter
                            elide: Text.ElideRight
                        }
                    }
                }
            }

            Rectangle {
                width: parent.width
                height: outputSignalColumn.implicitHeight + 32
                radius: 12
                color: "#111923"
                border.color: "#33485d"
                border.width: 1

                Column {
                    id: outputSignalColumn
                    x: 16
                    y: 16
                    width: parent.width - 32
                    spacing: 12

                    Text {
                        text: "Motor Output Signal"
                        color: "#e6ecf2"
                        font.pixelSize: 17
                        font.bold: true
                    }

                    GridLayout {
                        width: parent.width
                        columns: 4
                        columnSpacing: 8

                        ColumnLayout {
                            Layout.fillWidth: true
                            spacing: 2
                            Text { text: "PWM"; color: "#7890a3"; font.pixelSize: 11 }
                            Text {
                                text: currentPwmValue() + " / 255"
                                color: "#65b9f2"
                                font.pixelSize: 18
                                font.bold: true
                            }
                        }

                        ColumnLayout {
                            Layout.fillWidth: true
                            spacing: 2
                            Text { text: "DUTY CYCLE"; color: "#7890a3"; font.pixelSize: 11 }
                            Text {
                                text: currentDutyPercent().toFixed(1) + "%"
                                color: "#dce6ee"
                                font.pixelSize: 18
                                font.bold: true
                            }
                        }

                        ColumnLayout {
                            Layout.fillWidth: true
                            spacing: 2
                            Text { text: "SUPPLY"; color: "#7890a3"; font.pixelSize: 11 }
                            Text {
                                text: formatValue(supplyVoltage(), 2) + " V"
                                color: "#dce6ee"
                                font.pixelSize: 18
                                font.bold: true
                            }
                        }

                        ColumnLayout {
                            Layout.fillWidth: true
                            spacing: 2
                            Text { text: "AVG MOTOR"; color: "#7890a3"; font.pixelSize: 11 }
                            Text {
                                text: formatValue(estimatedAverageMotorVoltage(), 2) + " V"
                                color: testPowerPercent < 0 ? "#e68a8a" : (testPowerPercent > 0 ? "#79d3a6" : "#dce6ee")
                                font.pixelSize: 18
                                font.bold: true
                            }
                        }
                    }

                    Rectangle {
                        width: parent.width
                        height: 180
                        radius: 9
                        color: "#0b131d"
                        border.color: "#30465a"

                        Canvas {
                            id: pwmSignalGraph
                            anchors.fill: parent
                            anchors.margins: 8

                            property int pwmValue: currentPwmValue()
                            property real signalVoltage: supplyVoltage()
                            property real averageVoltage: Math.abs(estimatedAverageMotorVoltage())
                            property int powerDirection: testPowerPercent < 0 ? -1 : (testPowerPercent > 0 ? 1 : 0)

                            onPwmValueChanged: requestPaint()
                            onSignalVoltageChanged: requestPaint()
                            onAverageVoltageChanged: requestPaint()
                            onPowerDirectionChanged: requestPaint()
                            onWidthChanged: requestPaint()
                            onHeightChanged: requestPaint()

                            onPaint: {
                                const ctx = getContext("2d")
                                ctx.reset()
                                ctx.clearRect(0, 0, width, height)

                                const left = 58
                                const right = 12
                                const top = 18
                                const bottom = 30
                                const plotWidth = Math.max(1, width - left - right)
                                const plotHeight = Math.max(1, height - top - bottom)
                                const lowY = top + plotHeight
                                const highY = top
                                const duty = Math.max(0, Math.min(1, pwmValue / 255.0))
                                const supply = Number.isFinite(signalVoltage) ? Math.abs(signalVoltage) : 0
                                const averageY = supply > 0
                                    ? lowY - plotHeight * Math.max(0, Math.min(1, averageVoltage / supply))
                                    : lowY

                                ctx.font = "10px sans-serif"
                                ctx.textAlign = "right"
                                ctx.textBaseline = "middle"
                                ctx.fillStyle = "#7890a3"
                                ctx.fillText(supply > 0 ? supply.toFixed(1) + " V" : "Supply", left - 7, highY)
                                ctx.fillText("0 V", left - 7, lowY)

                                ctx.strokeStyle = "#263746"
                                ctx.lineWidth = 1
                                ctx.beginPath()
                                ctx.moveTo(left, highY)
                                ctx.lineTo(left + plotWidth, highY)
                                ctx.moveTo(left, lowY)
                                ctx.lineTo(left + plotWidth, lowY)
                                ctx.stroke()

                                const periodCount = 6
                                const periodWidth = plotWidth / periodCount
                                ctx.strokeStyle = "#5bb7f0"
                                ctx.lineWidth = 2.5
                                ctx.beginPath()
                                ctx.moveTo(left, lowY)
                                if (duty <= 0) {
                                    ctx.lineTo(left + plotWidth, lowY)
                                } else if (duty >= 1) {
                                    ctx.lineTo(left, highY)
                                    ctx.lineTo(left + plotWidth, highY)
                                } else {
                                    for (let period = 0; period < periodCount; ++period) {
                                        const startX = left + period * periodWidth
                                        const pulseEndX = startX + periodWidth * duty
                                        const endX = startX + periodWidth
                                        ctx.lineTo(startX, lowY)
                                        ctx.lineTo(startX, highY)
                                        ctx.lineTo(pulseEndX, highY)
                                        ctx.lineTo(pulseEndX, lowY)
                                        ctx.lineTo(endX, lowY)
                                    }
                                }
                                ctx.stroke()

                                ctx.strokeStyle = "#79d3a6"
                                ctx.lineWidth = 1.5
                                ctx.beginPath()
                                ctx.moveTo(left, averageY)
                                ctx.lineTo(left + plotWidth, averageY)
                                ctx.stroke()

                                ctx.fillStyle = "#79d3a6"
                                ctx.textAlign = "left"
                                ctx.fillText(
                                    "Average " + (Number.isFinite(averageVoltage) ? averageVoltage.toFixed(2) : "N/A") + " V",
                                    left + 6,
                                    Math.max(top + 8, averageY - 9)
                                )
                                ctx.fillStyle = "#7890a3"
                                ctx.textAlign = "right"
                                ctx.textBaseline = "bottom"
                                ctx.fillText(
                                    powerDirection < 0 ? "Reverse polarity" : (powerDirection > 0 ? "Forward polarity" : "Motor stopped"),
                                    left + plotWidth,
                                    height - 2
                                )
                            }
                        }
                    }

                    Text {
                        width: parent.width
                        wrapMode: Text.WordWrap
                        text: "Estimated from battery telemetry and the configured PWM curve. The waveform shows motor-terminal voltage switching between 0 V and supply voltage; it is not a direct oscilloscope measurement."
                        color: "#8198aa"
                        font.pixelSize: 11
                    }
                }
            }

            Rectangle {
                width: parent.width
                height: calibrationColumn.implicitHeight + 32
                radius: 12
                color: "#111923"
                border.color: "#33485d"
                border.width: 1
                opacity: testRunning ? 0.55 : 1.0
                enabled: !testRunning

                Column {
                    id: calibrationColumn
                    x: 16
                    y: 16
                    width: parent.width - 32
                    spacing: 12

                    Text {
                        text: "Advanced Calibration"
                        color: "#e6ecf2"
                        font.pixelSize: 17
                        font.bold: true
                    }

                    Text {
                        width: parent.width
                        wrapMode: Text.WordWrap
                        text: "One calibration profile is used for both forward and reverse. Motor orientation only changes which electrical direction is treated as forward."
                        color: "#8fa6b8"
                        font.pixelSize: 12
                    }

                    Rectangle {
                        width: parent.width
                        height: orientationColumn.implicitHeight + 20
                        radius: 8
                        color: "#172332"
                        border.color: calibrationDefaultReversed ? "#d49a55" : "#3b5064"

                        Column {
                            id: orientationColumn
                            x: 10
                            y: 10
                            width: parent.width - 20
                            spacing: 2

                            Row {
                                width: parent.width
                                spacing: 10

                                Text {
                                    width: parent.width - 196
                                    text: "Default motor orientation"
                                    color: "#dce6ee"
                                    font.pixelSize: 13
                                    font.bold: true
                                    anchors.verticalCenter: parent.verticalCenter
                                }

                                Text {
                                    width: 50
                                    text: "NORMAL"
                                    color: calibrationDefaultReversed ? "#667b8c" : "#79d3a6"
                                    font.pixelSize: 10
                                    font.bold: true
                                    horizontalAlignment: Text.AlignRight
                                    anchors.verticalCenter: parent.verticalCenter
                                }

                                Switch {
                                    id: orientationSwitch
                                    width: 56
                                    height: 28
                                    checked: calibrationDefaultReversed
                                    onToggled: calibrationDefaultReversed = checked

                                    indicator: Rectangle {
                                        x: 4
                                        y: 3
                                        width: 48
                                        height: 22
                                        radius: 11
                                        color: orientationSwitch.checked ? "#b97837" : "#34495b"
                                        border.color: orientationSwitch.checked ? "#efb76f" : "#61798c"

                                        Rectangle {
                                            x: orientationSwitch.checked ? parent.width - width - 3 : 3
                                            y: 3
                                            width: 16
                                            height: 16
                                            radius: 8
                                            color: "#f2f6f9"

                                            Behavior on x { NumberAnimation { duration: 120 } }
                                        }
                                    }
                                }

                                Text {
                                    width: 50
                                    text: "REVERSED"
                                    color: calibrationDefaultReversed ? "#efb76f" : "#667b8c"
                                    font.pixelSize: 10
                                    font.bold: true
                                    anchors.verticalCenter: parent.verticalCenter
                                }
                            }

                            Text {
                                width: parent.width
                                text: calibrationDefaultReversed
                                    ? "Forward commands use reversed polarity"
                                    : "Forward commands use normal polarity"
                                color: calibrationDefaultReversed ? "#efb76f" : "#9fb2c1"
                                font.pixelSize: 12
                                elide: Text.ElideRight
                            }
                        }
                    }

                    GridLayout {
                        width: parent.width
                        columns: 4
                        columnSpacing: 10
                        rowSpacing: 9

                        Text { text: "Start PWM"; color: "#aebdca"; font.pixelSize: 13 }
                        SpinBox {
                            from: calibrationSustainMinPwm
                            to: calibrationMaxPwm
                            value: calibrationStartBoostPwm
                            editable: true
                            Layout.fillWidth: true
                            onValueModified: calibrationStartBoostPwm = value
                        }
                        Text { text: "Pulse (ms)"; color: "#aebdca"; font.pixelSize: 13 }
                        SpinBox {
                            from: 0
                            to: 5000
                            stepSize: 10
                            value: calibrationStartBoostMs
                            editable: true
                            Layout.fillWidth: true
                            onValueModified: calibrationStartBoostMs = value
                        }

                        Text { text: "Curve"; color: "#aebdca"; font.pixelSize: 13 }
                        SpinBox {
                            from: 50
                            to: 400
                            stepSize: 5
                            value: calibrationCurveTimes100
                            editable: true
                            Layout.fillWidth: true
                            textFromValue: function(value) { return (value / 100.0).toFixed(2) }
                            valueFromText: function(text) { return clamp(Math.round(Number(text) * 100), 50, 400) }
                            onValueModified: calibrationCurveTimes100 = value
                        }
                        Text { text: "Accel (sec)"; color: "#aebdca"; font.pixelSize: 13 }
                        SpinBox {
                            from: 1
                            to: 100
                            value: calibrationRampTenths
                            editable: true
                            Layout.fillWidth: true
                            textFromValue: function(value) { return (value / 10.0).toFixed(1) }
                            valueFromText: function(text) { return clamp(Math.round(Number(text) * 10), 1, 100) }
                            onValueModified: calibrationRampTenths = value
                        }
                    }

                    Text {
                        text: "Steady PWM range"
                        color: "#aebdca"
                        font.pixelSize: 13
                        font.bold: true
                    }

                    RangeSlider {
                        id: pwmRangeSlider
                        width: parent.width
                        from: 0
                        to: 255
                        stepSize: 1
                        first.value: calibrationSustainMinPwm
                        second.value: calibrationMaxPwm
                        first.onMoved: {
                            calibrationSustainMinPwm = Math.round(first.value)
                            normalizePwmValues()
                        }
                        second.onMoved: {
                            calibrationMaxPwm = Math.round(second.value)
                            normalizePwmValues()
                        }

                        background: Rectangle {
                            x: pwmRangeSlider.leftPadding
                            y: pwmRangeSlider.topPadding + pwmRangeSlider.availableHeight / 2 - height / 2
                            width: pwmRangeSlider.availableWidth
                            height: 8
                            radius: 4
                            color: "#273746"

                            Rectangle {
                                x: pwmRangeSlider.first.visualPosition * parent.width
                                width: (pwmRangeSlider.second.visualPosition - pwmRangeSlider.first.visualPosition) * parent.width
                                height: parent.height
                                radius: parent.radius
                                color: "#4b9de0"
                            }
                        }
                    }

                    Item {
                        width: parent.width
                        height: 26

                        Text {
                            x: clamp(
                                pwmRangeSlider.leftPadding + pwmRangeSlider.first.visualPosition * pwmRangeSlider.availableWidth - width / 2,
                                0,
                                parent.width - width
                            )
                            text: "Min " + calibrationSustainMinPwm
                            color: "#7fc0f2"
                            font.pixelSize: 12
                            font.bold: true
                        }

                        Text {
                            x: clamp(
                                pwmRangeSlider.leftPadding + pwmRangeSlider.second.visualPosition * pwmRangeSlider.availableWidth - width / 2,
                                0,
                                parent.width - width
                            )
                            text: "Max " + calibrationMaxPwm
                            color: "#7fc0f2"
                            font.pixelSize: 12
                            font.bold: true
                        }
                    }

                    Rectangle {
                        width: parent.width
                        height: 245
                        radius: 9
                        color: "#0b131d"
                        border.color: "#30465a"

                        Text {
                            x: 12
                            y: 9
                            text: "PWM Response"
                            color: "#dce6ee"
                            font.pixelSize: 14
                            font.bold: true
                        }

                        Text {
                            anchors.right: parent.right
                            anchors.rightMargin: 12
                            y: 10
                            text: "Steady output  •  Startup boost"
                            color: "#7891a5"
                            font.pixelSize: 11
                        }

                        Canvas {
                            id: pwmGraph
                            x: 8
                            y: 32
                            width: parent.width - 16
                            height: parent.height - 40

                            property int minimumPwm: calibrationSustainMinPwm
                            property int maximumPwm: calibrationMaxPwm
                            property int startPwm: calibrationStartBoostPwm
                            property int curveTimes100: calibrationCurveTimes100
                            property int currentPower: testPowerPercent

                            onMinimumPwmChanged: requestPaint()
                            onMaximumPwmChanged: requestPaint()
                            onStartPwmChanged: requestPaint()
                            onCurveTimes100Changed: requestPaint()
                            onCurrentPowerChanged: requestPaint()
                            onWidthChanged: requestPaint()
                            onHeightChanged: requestPaint()

                            onPaint: {
                                const ctx = getContext("2d")
                                ctx.reset()
                                ctx.clearRect(0, 0, width, height)

                                const left = 42
                                const right = 12
                                const top = 10
                                const bottom = 30
                                const plotWidth = Math.max(1, width - left - right)
                                const plotHeight = Math.max(1, height - top - bottom)
                                const xFor = function(percent) { return left + plotWidth * percent / 100.0 }
                                const yFor = function(pwm) { return top + plotHeight * (1.0 - pwm / 255.0) }

                                ctx.font = "10px sans-serif"
                                ctx.lineWidth = 1
                                ctx.textAlign = "right"
                                ctx.textBaseline = "middle"
                                for (let pwm = 0; pwm <= 192; pwm += 64) {
                                    const y = yFor(pwm)
                                    ctx.strokeStyle = "#213140"
                                    ctx.beginPath()
                                    ctx.moveTo(left, y)
                                    ctx.lineTo(left + plotWidth, y)
                                    ctx.stroke()
                                    ctx.fillStyle = "#71889b"
                                    ctx.fillText(String(pwm), left - 6, y)
                                }
                                const topY = yFor(255)
                                ctx.strokeStyle = "#213140"
                                ctx.beginPath()
                                ctx.moveTo(left, topY)
                                ctx.lineTo(left + plotWidth, topY)
                                ctx.stroke()
                                ctx.fillStyle = "#71889b"
                                ctx.fillText("255", left - 6, topY)

                                ctx.textAlign = "center"
                                ctx.textBaseline = "top"
                                for (let percent = 0; percent <= 100; percent += 25) {
                                    const x = xFor(percent)
                                    ctx.strokeStyle = "#1b2a38"
                                    ctx.beginPath()
                                    ctx.moveTo(x, top)
                                    ctx.lineTo(x, top + plotHeight)
                                    ctx.stroke()
                                    ctx.fillStyle = "#71889b"
                                    ctx.fillText(percent + "%", x, top + plotHeight + 7)
                                }

                                ctx.strokeStyle = "#efaa55"
                                ctx.lineWidth = 1.5
                                ctx.beginPath()
                                ctx.moveTo(left, yFor(startPwm))
                                ctx.lineTo(left + plotWidth, yFor(startPwm))
                                ctx.stroke()

                                ctx.strokeStyle = "#55aef0"
                                ctx.lineWidth = 3
                                ctx.beginPath()
                                ctx.moveTo(xFor(0), yFor(0))
                                for (let percent = 1; percent <= 100; percent += 1) {
                                    ctx.lineTo(xFor(percent), yFor(steadyPwmForPercent(percent)))
                                }
                                ctx.stroke()

                                if (currentPower !== 0) {
                                    const magnitude = Math.abs(currentPower)
                                    const markerX = xFor(magnitude)
                                    const markerY = yFor(steadyPwmForPercent(magnitude))
                                    ctx.fillStyle = "#79d3a6"
                                    ctx.beginPath()
                                    ctx.arc(markerX, markerY, 5, 0, Math.PI * 2)
                                    ctx.fill()
                                    ctx.strokeStyle = "#e8fff3"
                                    ctx.lineWidth = 1
                                    ctx.stroke()
                                }
                            }
                        }
                    }

                    Row {
                        spacing: 10

                        Button {
                            text: "Apply to Buoy"
                            enabled: isConnected()
                            onClicked: applyConfiguration()
                        }

                        Button {
                            text: "Reload from Buoy"
                            enabled: isConnected()
                            onClicked: requestConfiguration(true)
                        }
                    }

                    Text {
                        width: parent.width
                        wrapMode: Text.WordWrap
                        text: "Start PWM supplies the initial kick. Minimum is the lowest steady output, Curve shapes throttle response, and Accel controls ramp time."
                        color: "#8198aa"
                        font.pixelSize: 12
                    }
                }
            }

            Item { width: 1; height: 6 }
        }
    }

    Connections {
        target: backendObject

        function onStateChanged() {
            const connected = isConnected()
            if (connected && !configWasConnected) {
                configConnectionGeneration += 1
                configFetchIdSeen = 0
            }
            if (!connected && configWasConnected) {
                testPowerPercent = 0
                powerSlider.value = 0
                testRunning = false
            }
            configWasConnected = connected

            syncConfiguration()
            requestConfiguration(false)
        }
    }
}
