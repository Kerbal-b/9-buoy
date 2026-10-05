import QtQuick
import QtQuick.Controls.Basic

SpinBox {
    id: control

    implicitWidth: 116
    implicitHeight: 36
    leftPadding: 25
    rightPadding: 25
    palette.text: "#e6ecf2"
    palette.buttonText: "#d9e4ec"
    palette.base: "#17232e"

    background: Rectangle {
        radius: 5
        color: control.enabled ? "#17232e" : "#111923"
        border.color: control.activeFocus ? "#5aa8dc" : "#3b5064"
    }

    contentItem: TextInput {
        z: 2
        text: control.displayText
        font: control.font
        color: control.enabled ? "#e6ecf2" : "#71889b"
        selectionColor: "#28638d"
        selectedTextColor: "#ffffff"
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        readOnly: !control.editable
        validator: control.validator
        inputMethodHints: Qt.ImhDigitsOnly
    }

    down.indicator: Rectangle {
        x: 3
        y: (control.height - height) / 2
        width: 20
        height: control.height - 6
        radius: 3
        color: control.down.pressed ? "#29445a" : "transparent"
        Text { anchors.centerIn: parent; text: "-"; color: "#9eb5c7"; font.pixelSize: 15 }
    }

    up.indicator: Rectangle {
        x: control.width - width - 3
        y: (control.height - height) / 2
        width: 20
        height: control.height - 6
        radius: 3
        color: control.up.pressed ? "#29445a" : "transparent"
        Text { anchors.centerIn: parent; text: "+"; color: "#9eb5c7"; font.pixelSize: 15 }
    }
}
