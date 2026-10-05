import QtQuick
import QtQuick.Layouts

GridLayout {
    id: root
    objectName: "tx12MappingPanel"
    property var controls: []
    property var backendState: ({})
    readonly property real channelMinimum: 988
    readonly property real channelMaximum: 2012
    columns: 2
    columnSpacing: 8
    rowSpacing: 6

    function shortLabel(control) {
        const labels = {
            left_stick_horizontal: "Left stick H", left_stick_vertical: "Left stick V",
            right_stick_horizontal: "Right stick H", right_stick_vertical: "Right stick V"
        }
        return labels[control.id] || control.id.toUpperCase()
    }

    function isStick(control) {
        return control.id === "left_stick_horizontal" || control.id === "left_stick_vertical"
            || control.id === "right_stick_horizontal" || control.id === "right_stick_vertical"
    }

    function channelValue(control) {
        if (control.channel === null || control.channel === undefined) return null
        const channels = backendState.radioChannels || []
        const value = Number(channels[control.channel - 1])
        return Number.isFinite(value) && value >= 800 && value <= 2200 ? value : null
    }

    function channelFraction(value) {
        return value === null ? 0.5 : Math.max(0, Math.min(1, (value - channelMinimum) / (channelMaximum - channelMinimum)))
    }

    function positionIndex(control) {
        const value = channelValue(control)
        if (value === null || !control.positions || !control.positions.length) return -1
        return Math.min(control.positions.length - 1, Math.floor(channelFraction(value) * control.positions.length))
    }

    Repeater {
        model: root.controls
        delegate: Rectangle {
            id: controlCard
            required property var modelData
            readonly property var control: modelData
            readonly property var liveValue: root.channelValue(control)
            readonly property bool stick: root.isStick(control)
            Layout.fillWidth: true
            implicitHeight: 54
            radius: 6
            color: "#111c28"
            border.color: "#395166"

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 7
                spacing: 4
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 7
                    Text {
                        text: root.shortLabel(controlCard.control) + (controlCard.control.purpose && controlCard.control.purpose !== "Unassigned" ? "  ·  " + controlCard.control.purpose : "")
                        color: "#e6ecf2"; font.pixelSize: 11; font.bold: true
                        Layout.fillWidth: true; elide: Text.ElideRight
                    }
                    Text {
                        text: controlCard.control.channel === null ? "No channel" : "CH" + controlCard.control.channel + "  " + (controlCard.liveValue === null ? "--" : controlCard.liveValue + " us")
                        color: controlCard.liveValue === null ? "#829aad" : "#79d6cf"; font.pixelSize: 10
                    }
                }

                Item {
                    visible: controlCard.stick
                    Layout.fillWidth: true; Layout.preferredHeight: 13
                    Rectangle { anchors.fill: parent; radius: 4; color: "#263747" }
                    Rectangle {
                        visible: controlCard.liveValue !== null
                        x: controlCard.control.id === "left_stick_vertical" ? 0 : Math.min(parent.width / 2, root.channelFraction(controlCard.liveValue) * parent.width)
                        width: controlCard.control.id === "left_stick_vertical"
                            ? root.channelFraction(controlCard.liveValue) * parent.width
                            : Math.abs(root.channelFraction(controlCard.liveValue) - 0.5) * parent.width
                        height: parent.height; radius: 4; color: "#4caea7"
                    }
                    Rectangle {
                        visible: controlCard.control.id !== "left_stick_vertical"
                        x: parent.width / 2 - 1; width: 2; height: parent.height; color: "#b7ccd5"
                    }
                    Rectangle {
                        visible: controlCard.liveValue !== null
                        x: Math.max(0, Math.min(parent.width - width, root.channelFraction(controlCard.liveValue) * parent.width - width / 2))
                        y: -2; width: 5; height: parent.height + 4; radius: 2; color: "#e4fcf8"
                    }
                    Text { anchors.left: parent.left; anchors.verticalCenter: parent.verticalCenter; anchors.leftMargin: 4; text: controlCard.control.id === "left_stick_vertical" ? "0" : "-"; color: "#e6ecf2"; font.pixelSize: 9 }
                    Text { anchors.right: parent.right; anchors.verticalCenter: parent.verticalCenter; anchors.rightMargin: 4; text: controlCard.control.id === "left_stick_vertical" ? "100%" : "+"; color: "#e6ecf2"; font.pixelSize: 9 }
                }

                RowLayout {
                    visible: Boolean(controlCard.control.positions && controlCard.control.positions.length)
                    Layout.fillWidth: true; spacing: 4
                    Repeater {
                        model: controlCard.control.positions || []
                        delegate: Rectangle {
                            required property var modelData
                            required property int index
                            Layout.fillWidth: true; implicitHeight: 19; radius: 4
                            color: root.positionIndex(controlCard.control) === index ? "#28776f" : "#263747"
                            border.color: root.positionIndex(controlCard.control) === index ? "#9ce8e3" : "#395166"
                            Text {
                                anchors.fill: parent; anchors.leftMargin: 3; anchors.rightMargin: 3
                                verticalAlignment: Text.AlignVCenter; horizontalAlignment: Text.AlignHCenter
                                text: modelData.label + (modelData.meaning ? ": " + modelData.meaning : "")
                                color: "#e6ecf2"; font.pixelSize: 9; elide: Text.ElideRight
                            }
                        }
                    }
                }
                Text {
                    visible: !controlCard.stick && !(controlCard.control.positions && controlCard.control.positions.length)
                    text: "No function mapped"
                    color: "#829aad"; font.pixelSize: 10
                }
            }
        }
    }
}
