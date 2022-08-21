import QtQuick 2.15
import QtQuick.Controls 2.15


Rectangle {
    width: 800 * root.width / 2560
    height: root.height
    color: "#343434"
    border.color: "#ffffff"

    Text {
        x: parent.x
        y: parent.y
        width: parent.width

        color: "#ffffff"
        text: "Settings"
        horizontalAlignment: Text.AlignHCenter
        font.pointSize: 30 * root.width/ 2560
        font.styleName: "Bold"

    }
    Button {
        id: settings_toggle_button
        anchors{
            left: parent.left
            top: parent.top
            topMargin: parent.height/2 - 128 * root.height/ 1600
        }
        icon.name: "settings"
        icon.source: "./images/close-window-128.gif"
        icon.color: "#620b66"
        icon.width: 64* root.width/ 2560
        icon.height: 64* root.height/ 1600


        onClicked: {stackview_settings.pop();
                    animation_right_reverse.running = true;
                    animation_bottom_reverse.running = true;
                    }
        background: Rectangle {
            id: power_button_bg
            color: parent.down? "#b1b1b1" : "#00fbfbfb"
            radius: 10
            border.color: "#3afbfbfb"

        }

    }
}

