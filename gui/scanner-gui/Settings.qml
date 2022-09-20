import QtQuick 2.1
import QtQuick.Controls 2.1
import QtQuick.LocalStorage 2.1
Rectangle {
    id: root
    width: 240
    height: 480

    visible: true
    color: "#000000"

    Loader {
        id: settingsLoader
        anchors{
            left: parent.left
            right: parent.right
            top: parent.top
            bottom: parent.bottom
        }

        NumberAnimation {
                    id: animation_right
                    target: settingsLoader.item
                    property: "anchors.rightMargin"
                    from: 0
                    to: root.width
                    duration: 100
                    easing.type: Easing.InExpo
                }
        NumberAnimation {
                    id: animation_bottom
                    target: settingsLoader.item
                    property: "anchors.bottomMargin"
                    from: 0
                    to: root.height
                    duration: 100
                    easing.type: Easing.InExpo
                }
        NumberAnimation {
                    id: animation_bottom_reverse
                    target: settingsLoader.item
                    property: "anchors.bottomMargin"
                    from: root.height
                    to: 0
                    duration: 100
                    easing.type: Easing.InExpo
                }
        NumberAnimation {
                    id: animation_right_reverse
                    target: settingsLoader.item
                    property: "anchors.rightMargin"
                    from: root.width
                    to: 0
                    duration: 100
                    easing.type: Easing.InExpo
                }

        source: "StackViewSettings.qml"
    }
}
