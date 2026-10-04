import QtQuick
import QtQuick.Controls

Item {
    id: root
    property var backendObject
    property var backendState: ({})

    function selectedChannelIndex() {
        if (!backendObject) return 0
        if (backendObject.audioDebugChannel === "left") return 1
        if (backendObject.audioDebugChannel === "right") return 2
        return 0
    }

    function selectedQualityIndex() {
        if (!backendObject || backendObject.audioQuality === "DEFAULT") return 1
        if (backendObject.audioQuality === "MAX") return 2
        return 0
    }

    Rectangle {
        anchors.fill: parent
        radius: 12
        color: "#0b1119"
        border.color: "#395166"

        Column {
            anchors.fill: parent
            anchors.margins: 22
            spacing: 16

            Row {
                width: parent.width
                height: 58
                spacing: 14
                Column {
                    width: parent.width - 185
                    Text { text: "Science Instrument Debugging"; color: "#e6ecf2"; font.pixelSize: 23; font.bold: true }
                    Text { text: "Instrument 1 | INMP441 microphone pair"; color: "#829aad"; font.pixelSize: 13 }
                }
                Rectangle {
                    width: 170; height: 46; radius: 9
                    color: backendState.audioStreamEnabled && Number(backendState.audioChannelCount || 0) === 2 ? "#17392e" : "#34291b"
                    border.color: backendState.audioStreamEnabled && Number(backendState.audioChannelCount || 0) === 2 ? "#3e9c73" : "#bd8d43"
                    Text {
                        anchors.centerIn: parent
                        text: !backendState.audioStreamEnabled ? "AUDIO STREAM OFF" : (Number(backendState.audioChannelCount || 0) === 2 ? "2 CHANNEL STREAM" : "WAITING FOR AUDIO")
                        color: "#e6ecf2"; font.pixelSize: 11; font.bold: true
                    }
                }
            }

            Row {
                width: parent.width
                height: parent.height - 460
                spacing: 16

                Repeater {
                    model: [
                        { title: "LEFT I2S CHANNEL", key: "left", color: "#55b8f2" },
                        { title: "RIGHT I2S CHANNEL", key: "right", color: "#c58aff" }
                    ]
                    delegate: Rectangle {
                        property bool streamActive: backendState.audioStreamEnabled === true
                        width: (parent.width - 16) / 2
                        height: parent.height
                        radius: 10
                        color: "#101a25"
                        border.color: "#30465a"

                        Column {
                            anchors.fill: parent
                            anchors.margins: 16
                            spacing: 10
                            Row {
                                width: parent.width
                                Text { text: modelData.title; color: "#9eb5c7"; font.pixelSize: 12; font.bold: true }
                                Item { width: parent.width - 230; height: 1 }
                                Text {
                                    text: modelData.key === "left" ? (backendObject ? backendObject.audioLeftLevel : "N/A") : (backendObject ? backendObject.audioRightLevel : "N/A")
                                    color: modelData.color; font.pixelSize: 13; font.bold: true
                                }
                            }
                            Canvas {
                                id: waveformCanvas
                                width: parent.width
                                height: parent.height - 100
                                onWidthChanged: requestPaint()
                                onHeightChanged: requestPaint()
                                onPaint: {
                                    const ctx = getContext("2d")
                                    ctx.reset(); ctx.clearRect(0, 0, width, height)
                                    ctx.fillStyle = "#09121b"; ctx.fillRect(0, 0, width, height)
                                    if (!streamActive) {
                                        ctx.fillStyle = "#efb76f"; ctx.font = "13px sans-serif"
                                        ctx.fillText("Audio stream stopped", 12, 22); return
                                    }
                                    ctx.strokeStyle = "#213548"; ctx.lineWidth = 1
                                    for (let row = 0; row <= 4; ++row) {
                                        const y = row * height / 4
                                        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(width, y); ctx.stroke()
                                    }
                                    ctx.strokeStyle = "#47647b"
                                    ctx.beginPath(); ctx.moveTo(0, height / 2); ctx.lineTo(width, height / 2); ctx.stroke()
                                    const values = backendObject
                                        ? (modelData.key === "left" ? backendObject.audioLeftWaveform : backendObject.audioRightWaveform)
                                        : []
                                    if (!values || values.length < 2) {
                                        ctx.fillStyle = "#efb76f"; ctx.font = "13px sans-serif"
                                        ctx.fillText("Waiting for channel samples", 12, 22); return
                                    }
                                    ctx.strokeStyle = modelData.color; ctx.lineWidth = 2; ctx.beginPath()
                                    const step = Math.max(1, Math.floor(values.length / Math.max(2, width - 12)))
                                    let x = 6
                                    for (let i = 0; i < values.length; i += step) {
                                        const y = height / 2 - Math.max(-1, Math.min(1, values[i])) * height * 0.42
                                        if (x === 6) ctx.moveTo(x, y); else ctx.lineTo(x, y)
                                        x += 1
                                        if (x >= width - 6) break
                                    }
                                    ctx.stroke()
                                }
                            }
                            Row {
                                width: parent.width
                                Text {
                                    width: parent.width
                                    wrapMode: Text.WordWrap
                                    text: modelData.key === "left"
                                        ? "This trace is the I2S left slot (INMP441 L/R pin low). Tap only the left mic to confirm which physical board feeds it."
                                        : "This trace is the I2S right slot (INMP441 L/R pin high). Tap only the right mic to confirm which physical board feeds it."
                                    color: "#829aad"; font.pixelSize: 12
                                }
                            }
                        }
                        onStreamActiveChanged: waveformCanvas.requestPaint()
                        Timer {
                            interval: 100
                            repeat: true
                            running: root.visible && streamActive
                            onTriggered: waveformCanvas.requestPaint()
                        }
                    }
                }
            }

            Rectangle {
                width: parent.width
                height: 72
                radius: 10
                color: "#111c28"
                border.color: "#30465a"
                Row {
                    anchors.fill: parent
                    anchors.margins: 12
                    spacing: 12
                    Text { text: "Firmware capture"; color: "#9eb5c7"; anchors.verticalCenter: parent.verticalCenter }
                    ComboBox {
                        width: 170
                        anchors.verticalCenter: parent.verticalCenter
                        model: ["Both microphones", "Left only", "Right only"]
                        currentIndex: !backendObject || backendObject.audioCaptureChannel === "BOTH" ? 0
                            : (backendObject.audioCaptureChannel === "LEFT" ? 1 : 2)
                        onActivated: function(index) {
                            if (backendObject) backendObject.setAudioCaptureChannel(["BOTH", "LEFT", "RIGHT"][index])
                        }
                    }
                    Text { text: "Quality"; color: "#9eb5c7"; anchors.verticalCenter: parent.verticalCenter }
                    ComboBox {
                        width: 190
                        anchors.verticalCenter: parent.verticalCenter
                        model: ["Low | 8 kHz", "Default | 16 kHz", "Max | 48 kHz"]
                        currentIndex: root.selectedQualityIndex()
                        enabled: backendObject && !backendObject.audioRecordingEnabled && !backendObject.audioQualityLocked
                        onActivated: function(index) {
                            if (backendObject) backendObject.setAudioQuality(["LOW", "DEFAULT", "MAX"][index])
                        }
                    }
                    Button {
                        text: backendObject && backendObject.audioRecordingEnabled ? "Stop SD audio" : "Start SD audio"
                        enabled: backendObject && (backendObject.audioRecordingEnabled || backendState.scienceExperimentState === "RECORDING")
                        anchors.verticalCenter: parent.verticalCenter
                        onClicked: if (backendObject) backendObject.toggleAudioRecording()
                    }
                    Text {
                        text: "SD: " + (backendObject ? backendObject.audioSdStatus : "Unknown")
                        color: backendObject && backendObject.audioSdStatus === "READY" ? "#72d6a4" : "#efb76f"
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    Text {
                        text: backendObject && backendObject.audioQualityLocked
                            ? "Quality locked until this experiment is saved and closed."
                            : "Sample rate does not reduce microphone self-noise."
                        color: "#829aad"; font.pixelSize: 11; wrapMode: Text.WordWrap
                        width: Math.max(120, parent.width - 850)
                        anchors.verticalCenter: parent.verticalCenter
                    }
                }
            }

            Rectangle {
                width: parent.width
                height: 82
                radius: 10
                color: "#111c28"
                border.color: "#30465a"
                Row {
                    anchors.fill: parent
                    anchors.margins: 14
                    spacing: 16
                    Text { text: "Headphone monitor"; color: "#9eb5c7"; anchors.verticalCenter: parent.verticalCenter }
                    ComboBox {
                        width: 155
                        anchors.verticalCenter: parent.verticalCenter
                        model: ["Stereo", "Left to both ears", "Right to both ears"]
                        currentIndex: root.selectedChannelIndex()
                        onActivated: function(index) {
                            if (backendObject) backendObject.setAudioDebugChannel(["both", "left", "right"][index])
                        }
                    }
                    Text { text: "Playback gain"; color: "#9eb5c7"; anchors.verticalCenter: parent.verticalCenter }
                    Slider {
                        width: Math.max(100, parent.width - 730)
                        from: 0; to: 200; stepSize: 5
                        value: backendObject ? backendObject.audioDebugGain : 100
                        anchors.verticalCenter: parent.verticalCenter
                        onMoved: if (backendObject) backendObject.setAudioDebugGain(value)
                    }
                    Text {
                        width: 56
                        text: Math.round(backendObject ? backendObject.audioDebugGain : 100) + "%"
                        color: "#e6ecf2"; anchors.verticalCenter: parent.verticalCenter
                    }
                    Button {
                        text: backendState.audioStreamEnabled ? "Stop audio stream" : "Start audio stream"
                        anchors.verticalCenter: parent.verticalCenter
                        onClicked: if (backendObject) backendObject.setAudioStreamEnabled(!backendState.audioStreamEnabled)
                    }
                    Button {
                        text: backendObject && backendObject.audioMuted ? "Unmute" : "Mute"
                        anchors.verticalCenter: parent.verticalCenter
                        onClicked: if (backendObject) backendObject.toggleAudioMute()
                    }
                }
            }

            Rectangle {
                width: parent.width
                height: 76
                radius: 10
                color: "#111c28"
                border.color: "#30465a"
                Column {
                    anchors.fill: parent
                    anchors.margins: 8
                    spacing: 2
                    Row {
                        width: parent.width
                        height: 28
                        spacing: 8
                        CheckBox {
                            text: "High pass"
                            checked: backendObject ? backendObject.audioHighPassEnabled : true
                            onToggled: if (backendObject) backendObject.setAudioHighPassEnabled(checked)
                        }
                        Text { text: "Cutoff"; color: "#9eb5c7"; anchors.verticalCenter: parent.verticalCenter }
                        Slider {
                            width: Math.max(90, parent.width * 0.15)
                            from: 20; to: 300; stepSize: 10
                            value: backendObject ? backendObject.audioHighPassHz : 80
                            anchors.verticalCenter: parent.verticalCenter
                            onMoved: if (backendObject) backendObject.setAudioHighPassHz(value)
                        }
                        Text { text: Math.round(backendObject ? backendObject.audioHighPassHz : 80) + " Hz"; color: "#e6ecf2"; anchors.verticalCenter: parent.verticalCenter }
                        CheckBox {
                            text: "Low pass"
                            checked: backendObject ? backendObject.audioLowPassEnabled : true
                            onToggled: if (backendObject) backendObject.setAudioLowPassEnabled(checked)
                        }
                        Slider {
                            width: Math.max(90, parent.width * 0.15)
                            from: 500; to: backendObject ? Math.max(500, Math.min(7900, backendObject.audioSampleRateHz * 0.45)) : 7900; stepSize: 100
                            value: backendObject ? Math.min(backendObject.audioLowPassHz, to) : 6000
                            anchors.verticalCenter: parent.verticalCenter
                            onMoved: if (backendObject) backendObject.setAudioLowPassHz(value)
                        }
                        Text { text: Math.round(backendObject ? backendObject.audioLowPassHz : 6000) + " Hz"; color: "#e6ecf2"; anchors.verticalCenter: parent.verticalCenter }
                    }
                    Row {
                        width: parent.width
                        height: 28
                        spacing: 8
                        CheckBox {
                            text: "Noise gate"
                            checked: backendObject ? backendObject.audioNoiseGateEnabled : false
                            onToggled: if (backendObject) backendObject.setAudioNoiseGateEnabled(checked)
                        }
                        Text { text: "Gate threshold"; color: "#9eb5c7"; anchors.verticalCenter: parent.verticalCenter }
                        Slider {
                            width: Math.max(130, parent.width * 0.25)
                            from: 0; to: 10; stepSize: 0.5
                            value: backendObject ? backendObject.audioNoiseGatePercent : 2.5
                            anchors.verticalCenter: parent.verticalCenter
                            onMoved: if (backendObject) backendObject.setAudioNoiseGatePercent(value)
                        }
                        Text { text: Number(backendObject ? backendObject.audioNoiseGatePercent : 2.5).toFixed(1) + "%"; color: "#e6ecf2"; anchors.verticalCenter: parent.verticalCenter }
                        Text { text: "Filtering applies to this laptop's trace and monitor only."; color: "#829aad"; anchors.verticalCenter: parent.verticalCenter; font.pixelSize: 11 }
                    }
                }
            }

            Rectangle {
                width: parent.width
                height: 52
                radius: 7
                color: "#101a25"
                Column {
                    anchors.left: parent.left; anchors.leftMargin: 10
                    anchors.verticalCenter: parent.verticalCenter
                    spacing: 4
                    Text {
                        text: backendObject ? backendObject.audioPacketLossStatus : "Waiting for audio packets"
                        color: "#c2d0dc"; font.pixelSize: 12
                    }
                    Row {
                        spacing: 8
                        Text { text: "Output"; color: "#9eb5c7"; font.pixelSize: 11; anchors.verticalCenter: parent.verticalCenter }
                        ComboBox {
                            width: 270
                            height: 28
                            model: backendObject ? backendObject.audioOutputDevices : ["System default"]
                            currentIndex: backendObject
                                ? Math.max(0, backendObject.audioOutputDevices.indexOf(backendObject.audioOutputDevice))
                                : 0
                            onActivated: function(index) {
                                if (backendObject) backendObject.setAudioOutputDevice(backendObject.audioOutputDevices[index])
                            }
                        }
                        Text {
                            text: backendObject ? backendObject.audioOutputStatus : "Audio output unavailable"
                            color: "#9eb5c7"; font.pixelSize: 11; anchors.verticalCenter: parent.verticalCenter
                        }
                    }
                }
            }

            Text {
                width: parent.width
                text: "Waveforms auto-scale for visibility; peak values remain unscaled. Tap each mic separately and watch its channel peak. Filters affect laptop playback and traces only. Local playback drops mean the laptop buffer overflowed; missing packets are counted separately."
                color: "#829aad"; font.pixelSize: 11; wrapMode: Text.WordWrap
            }
        }
    }
}
