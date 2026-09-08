import QtQuick

Item {
    id: root

    property var backendObject
    property var backendState: ({})

    Rectangle {
        anchors.fill: parent
        radius: 12
        color: "#0b1119"
        border.color: "#395166"
        border.width: 1

        Text {
            x: 20
            y: 18
            text: "Science & Audio"
            color: "#e6ecf2"
            font.pixelSize: 22
            font.bold: true
        }

        Text {
            x: 20
            y: 48
            text: "Live environmental, motion, and acoustic telemetry"
            color: "#829aad"
            font.pixelSize: 12
        }

        Row {
            x: 20
            y: 82
            width: parent.width - 40
            height: parent.height - 102
            spacing: 18

            Column {
                width: Math.min(360, parent.width * 0.38)
                spacing: 8

                Text {
                    text: "SENSOR READINGS"
                    color: "#7890a3"
                    font.pixelSize: 11
                    font.bold: true
                }

                Repeater {
                    model: [
                        { label: "Depth", value: backendState.currentDepth },
                        { label: "Water temperature", value: backendState.waterTemperature },
                        { label: "Air temperature", value: backendState.airTemperature },
                        { label: "IMU acceleration", value: backendState.imuAccel },
                        { label: "IMU rotation", value: backendState.imuGyro },
                        { label: "IMU temperature", value: backendState.imuTemperature },
                        { label: "IMU UDP loss", value: backendState.imuUdpLoss }
                    ]

                    delegate: Rectangle {
                        width: parent.width
                        height: 48
                        radius: 8
                        color: "#111c28"
                        border.color: "#2f4356"

                        Text {
                            x: 12
                            anchors.verticalCenter: parent.verticalCenter
                            text: modelData.label
                            color: "#9eb5c7"
                            font.pixelSize: 13
                        }

                        Text {
                            anchors.right: parent.right
                            anchors.rightMargin: 12
                            anchors.verticalCenter: parent.verticalCenter
                            width: parent.width * 0.55
                            text: modelData.value || "N/A"
                            color: "#e6ecf2"
                            font.pixelSize: 13
                            font.bold: true
                            horizontalAlignment: Text.AlignRight
                            elide: Text.ElideRight
                        }
                    }
                }
            }

            Rectangle {
                width: parent.width - 378
                height: parent.height
                radius: 12
                color: "#0f1722"
                border.color: "#395166"
                border.width: 1

                Text {
                    x: 16
                    y: 14
                    text: "AUDIO STREAM"
                    color: "#7890a3"
                    font.pixelSize: 11
                    font.bold: true
                }

                Text {
                    x: 16
                    y: 40
                    width: parent.width - 110
                    text: "Stream: " + (backendState.audioStream || "N/A")
                    color: "#e6ecf2"
                    font.pixelSize: 13
                    elide: Text.ElideRight
                }

                Text {
                    x: 16
                    y: 64
                    text: "Level: " + (backendState.audioLevel || "N/A")
                    color: "#e6ecf2"
                    font.pixelSize: 13
                }

                Rectangle {
                    id: muteButton
                    anchors.right: parent.right
                    anchors.rightMargin: 16
                    y: 16
                    width: 64
                    height: 56
                    radius: 9
                    color: backendObject && backendObject.audioMuted ? "#b97837" : "#277a5b"
                    border.color: backendObject && backendObject.audioMuted ? "#efb76f" : "#79d3a6"
                    border.width: 1

                    Text {
                        anchors.centerIn: parent
                        text: backendObject && backendObject.audioMuted ? "MUTED" : "LIVE"
                        color: "#f0f5f8"
                        font.pixelSize: 11
                        font.bold: true
                    }

                    MouseArea {
                        anchors.fill: parent
                        cursorShape: Qt.PointingHandCursor
                        onClicked: {
                            if (backendObject) backendObject.toggleAudioMute()
                        }
                    }
                }

                Canvas {
                    id: audioCanvas
                    x: 16
                    y: 100
                    width: parent.width - 32
                    height: parent.height - 116

                    onWidthChanged: requestPaint()
                    onHeightChanged: requestPaint()

                    onPaint: {
                        const ctx = getContext("2d")
                        ctx.reset()
                        ctx.clearRect(0, 0, width, height)
                        ctx.fillStyle = "#0a121b"
                        ctx.fillRect(0, 0, width, height)

                        ctx.strokeStyle = "#1f3344"
                        ctx.lineWidth = 1
                        for (let row = 0; row <= 4; ++row) {
                            const y = row * height / 4
                            ctx.beginPath()
                            ctx.moveTo(0, y)
                            ctx.lineTo(width, y)
                            ctx.stroke()
                        }
                        for (let column = 0; column <= 8; ++column) {
                            const x = column * width / 8
                            ctx.beginPath()
                            ctx.moveTo(x, 0)
                            ctx.lineTo(x, height)
                            ctx.stroke()
                        }

                        ctx.strokeStyle = "#47647b"
                        ctx.beginPath()
                        ctx.moveTo(0, height / 2)
                        ctx.lineTo(width, height / 2)
                        ctx.stroke()

                        const waveform = backendObject ? backendObject.audioWaveform : []
                        if (!waveform || waveform.length < 2) {
                            ctx.fillStyle = "#efb76f"
                            ctx.font = "13px sans-serif"
                            ctx.fillText("Waiting for audio samples", 14, 24)
                            return
                        }

                        const step = Math.max(1, Math.floor(waveform.length / Math.max(2, width - 20)))
                        const scaleY = height * 0.42
                        ctx.strokeStyle = "#55b8f2"
                        ctx.lineWidth = 2
                        ctx.beginPath()
                        let drawIndex = 0
                        for (let index = 0; index < waveform.length; index += step) {
                            const x = 10 + drawIndex
                            if (x >= width - 10) break
                            const value = Math.max(-1.0, Math.min(1.0, waveform[index]))
                            const y = height / 2 - value * scaleY
                            if (drawIndex === 0) ctx.moveTo(x, y)
                            else ctx.lineTo(x, y)
                            drawIndex++
                        }
                        ctx.stroke()
                    }
                }
            }
        }
    }

    Connections {
        target: backendObject

        function onStateChanged() {
            audioCanvas.requestPaint()
        }
    }
}
