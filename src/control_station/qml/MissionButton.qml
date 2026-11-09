import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts

Button {
    id: control
    implicitHeight: 38
    Layout.minimumHeight: 38
    leftPadding: 12
    rightPadding: 12
    topPadding: 7
    bottomPadding: 7
    font.pixelSize: 12
    font.weight: Font.DemiBold
    palette.buttonText: "#e6ecf2"

    contentItem: Text {
        text: control.text
        font: control.font
        color: control.enabled ? control.palette.buttonText : "#788b9b"
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        elide: Text.ElideRight
    }

    background: Rectangle {
        radius: 6
        color: !control.enabled ? "#17232e"
            : control.down ? "#244a63"
            : control.checked ? "#28516a"
            : control.hovered ? "#21394b" : "#1b2d3c"
        border.color: control.enabled ? "#55748d" : "#2d3e4b"
    }
}
