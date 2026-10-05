import QtQuick
import QtQuick.Controls

CheckBox {
    id: control

    palette.windowText: "#e6ecf2"
    palette.text: "#e6ecf2"
    palette.buttonText: "#e6ecf2"

    contentItem: Text {
        leftPadding: control.indicator.width + control.spacing
        text: control.text
        font: control.font
        color: control.enabled ? "#e6ecf2" : "#829aad"
        elide: Text.ElideRight
        verticalAlignment: Text.AlignVCenter
    }
}
