import QtQuick 2.15
import QtQuick.Controls 2.15

Item {
    anchors.fill: parent
    StackView {
        id: stackview_settings
        anchors.fill: parent
        initialItem: "SettingsUI.qml"
        signal pagePopedPushed(string str)
/*
        pushEnter: Transition {

            PropertyAnimation {
                property: "opacity"
                from: 0
                to:1
                duration: 300
            }
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

            PropertyAnimation {
                property: "opacity"
                from: 0
                to:1
                duration: 300
            }

        }
        popExit: Transition {
            PropertyAnimation {
                property: "opacity"
                from: 1
                to:0
                duration: 300
            }
        }
        */

    }
}

/*##^##
Designer {
    D{i:0;autoSize:true;height:480;width:640}
}
##^##*/
