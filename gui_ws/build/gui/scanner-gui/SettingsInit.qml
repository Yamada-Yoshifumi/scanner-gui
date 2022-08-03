import QtQuick 2.15
import QtQuick.Controls 2.15

Rectangle {
    width: 800 * root.width / 2560
    height: root.height
    visible: false
    color: "#00ffffff"
    border.color: "#00ffffff"

    Button {
        id: settings_toggle_button
        anchors{
            right: parent.right
            top: parent.top
            topMargin: parent.height/2 - 128 * root.height/ 1600
        }
        icon.name: "settings"
        icon.source: "./images/settings-17-128.gif"
        icon.color: "#620b66"
        icon.width: 64* root.width/ 2560
        icon.height: 64* root.height/ 1600


        onClicked: {stackview_settings.push( "SettingsUI.qml" );
                    animation_right.running = true;
                    animation_bottom.running = true;
                    //mainLoader.anchors.bottomMargin = settingsLoader.width * root.height/ root.width;

                 }

        background: Rectangle {
            id: power_button_bg
            color: parent.down? "#b1b1b1" : "#00fbfbfb"
            radius: 10
            border.color: "#3afbfbfb"

        }

    }
}
