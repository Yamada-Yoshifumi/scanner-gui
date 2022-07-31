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
