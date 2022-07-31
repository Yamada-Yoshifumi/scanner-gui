import QtQuick 2.0
import QtQuick.Controls 2.12
import UntitledProject

Item {
    anchors.fill: parent
    StackView {
        id: stackview_settings
        anchors.fill: parent
        initialItem: "SettingsUI.qml"
    }
}

/*##^##
Designer {
    D{i:0;autoSize:true;height:480;width:640}
}
##^##*/
