import QtQuick 2.15
import QtQuick.Controls 2.15

Item {
    anchors.fill: parent
    StackView {
        id: stackview
        anchors {
            top: parent.top
            left: parent.left
            right: parent.right
            bottom: parent.bottom
        }
        initialItem: "Bootpage.qml"
    }
}
