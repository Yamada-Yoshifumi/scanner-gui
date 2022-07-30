import QtQuick 2.15
import QtQuick.Controls 2.15

Window {
    id: root
    width: 2560
    height: 1600
    minimumHeight: 720
    minimumWidth: 1280
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
}


