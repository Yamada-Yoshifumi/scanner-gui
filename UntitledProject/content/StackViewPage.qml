import QtQuick 2.0
import QtQuick.Controls 2.12
import UntitledProject

Item {
    anchors.fill: parent
    StackView {
        id: stackview
        anchors.fill: parent
        initialItem: "Bootpage.qml"
    }
}
