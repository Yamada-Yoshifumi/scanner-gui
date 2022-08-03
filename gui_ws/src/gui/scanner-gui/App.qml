import QtQuick 2.15
import QtQuick.Controls 2.15

Window {
    id: root
    width: 2560
    height: 1600
    minimumHeight: 480
    minimumWidth: 640
    visible: true
    color: "#000000"
    title: "ScannerGUI"

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
                        to: settingsLoader.width
                        duration: 300
                        easing.type: Easing.InExpo
                    }
            NumberAnimation {
                        id: animation_bottom
                        target: mainLoader.item
                        property: "anchors.bottomMargin"
                        from: 0
                        to: settingsLoader.width * root.height/ root.width
                        duration: 300
                        easing.type: Easing.InExpo
                    }
            NumberAnimation {
                        id: animation_bottom_reverse
                        target: mainLoader.item
                        property: "anchors.bottomMargin"
                        from: settingsLoader.width * root.height/ root.width
                        to: 0
                        duration: 300
                        easing.type: Easing.InExpo
                    }
            NumberAnimation {
                        id: animation_right_reverse
                        target: mainLoader.item
                        property: "anchors.rightMargin"
                        from: settingsLoader.width
                        to: 0
                        duration: 300
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
