import QtQuick 2.15
import QtQuick.Controls 2.15

Rectangle {
    id: root
    width: 1280
    height: 720

    visible: true
    color: "#000000"

    Loader {
            id: mainLoader
            anchors {
                left: parent.left
                right: parent.right
                top: parent.top
                bottom: parent.bottom
            }
            source: "StackViewPage.qml"

            NumberAnimation {
                        id: animation_right
                        target: mainLoader.item
                        property: "anchors.rightMargin"
                        from: 0
                        to: root.width * 1 / 2
                        duration: 100
                        easing.type: Easing.InExpo
                    }
            NumberAnimation {
                        id: animation_bottom
                        target: mainLoader.item
                        property: "anchors.bottomMargin"
                        from: 0
                        to: root.height * 1 / 2
                        duration: 100
                        easing.type: Easing.InExpo
                    }
            NumberAnimation {
                        id: animation_bottom_reverse
                        target: mainLoader.item
                        property: "anchors.bottomMargin"
                        from: root.height * 1 / 2
                        to: 0
                        duration: 100
                        easing.type: Easing.InExpo
                    }
            NumberAnimation {
                        id: animation_right_reverse
                        target: mainLoader.item
                        property: "anchors.rightMargin"
                        from: root.width * 1 / 2
                        to: 0
                        duration: 100
                        easing.type: Easing.InExpo
                    }

    }

    Loader {
        id: settingsLoader
        anchors{
            left: parent.left
            right: parent.right
            top: parent.top
            bottom: parent.bottom
            leftMargin: 1800 * root.width / 2560;
        }
        source: "StackViewSettings.qml"
    }

}



/*##^##
Designer {
    D{i:0;formeditorZoom:0.125}
}
##^##*/
