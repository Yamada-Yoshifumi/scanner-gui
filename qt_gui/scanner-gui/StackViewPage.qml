import QtQuick 2.1
import QtQuick.Controls 2.1

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
        initialItem: "Mainui.qml"
    }
}
