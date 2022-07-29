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

    Bootpage{
        id: bootpage
        width: parent.width
        height: parent.height
    }

    MainUI{
        id: mainpage
        width: parent.width
        height:parent.height
    }
}

/*##^##
Designer {
    D{i:0;formeditorZoom:0.2}
}
##^##*/
