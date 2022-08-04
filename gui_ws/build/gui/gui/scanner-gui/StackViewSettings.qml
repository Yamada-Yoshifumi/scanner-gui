import QtQuick 2.0
import QtQuick.Controls 2.12

Item {
    anchors.fill: parent
    StackView {
        id: stackview_settings
        anchors.fill: parent
        initialItem: "SettingsInit.qml"
        /*
        pushEnter: Transition {

                duration: 300

        }
        pushExit: Transition {
            PropertyAnimation {
                property: "opacity"
                from: 1
                to:0
                duration: 300
            }
        }
        popEnter: Transition {

                duration: 300

        }
        popExit: Transition {
            PropertyAnimation {
                property: "opacity"
                from: 1
                to:0
                duration: 300
            }
        }*/
    }
}

/*##^##
Designer {
    D{i:0;autoSize:true;height:480;width:640}
}
##^##*/
