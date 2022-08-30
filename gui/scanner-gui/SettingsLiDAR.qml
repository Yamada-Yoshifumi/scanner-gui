import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.LocalStorage 2.15
import "model_handler.js" as JS

Rectangle {
    id: settings_lidar
    width: parent.width
    height: parent.height
    color: "#442e5d"
    border.color: "#442e5d"

    Text {
        id: settings_lidar_header
        x: parent.x
        y: parent.y
        width: parent.width
        height: parent.width / 5

        color: "#ffffff"
        text: "LiDAR"
        horizontalAlignment: Text.AlignHCenter
        font.pointSize: 100 * parent.width/ 2560
        font.styleName: "Bold"

    }

    ListModel {
            id: mvc_lidar_model
            property int display_reconstruction_value: 0
            /*
            ListElement {
                name: "Display Reconstruction"
                value: mvc_lidar_model.display_reconstruction_value
                cache: true
            }
            */
            property bool completed: false
            Component.onCompleted: {
                append({name: "Display Reconstruction", value: mvc_lidar_model.display_reconstruction_value});
                completed = true;
            }

            // 2. Update the list model:
            onDisplay_reconstruction_valueChanged:  {
                if(completed) setProperty(0, "value", mvc_lidar_model.display_reconstruction_value);
            }
        }

        // 2. Delegate - this describes how to handle each row of data
        Component {
            id: mvc_lidar_delegate
            Row {
                Text {
                    id: mvc_lidar_level_1
                    text: name
                    width: settings_lidar_header.width/2
                    height: settings_lidar_header.width/ 5
                    color: "#d4d4d4"
                    horizontalAlignment: Text.AlignHCenter
                    font.pointSize: 100 * parent.width/ 2560
                    MouseArea {
                        anchors.fill: parent
                        onClicked: {
                            mvc_lidar_listview.currentIndex = index
                            console.debug("Clicked on age")
                            JS.doSomething()  // Javascript from file
                        }
                    }
                }
                Switch {
                    position: value
                    checked: value
                    width: settings_lidar_header.width/2
                    height: settings_lidar_header.width/ 5
                    property bool init: false
                    MouseArea {
                        anchors.fill: parent
                        onClicked: {
                            value = value === 0? 1: 0;
                            var db = LocalStorage.openDatabaseSync("ScannerSettingsDB", "1.0", "Your QML SQL", 1000000);
                            db.transaction(
                                function(tx) {
                                    tx.executeSql('UPDATE BooleanSettings SET value = ? WHERE name="Display Reconstruction"', value);
                                }
                            )
                        }
                    }
                }
            }
        }


        // 3. ListView - this displays the rows as a list.
        Rectangle {
            // Put the ListView inside a rectangle for more layout control
            color: "#442e5d"

            anchors.top: settings_lidar_header.bottom
            anchors.bottom: settings_lidar.bottom
            width: settings_lidar.width
            ListView {
                id: mvc_lidar_listview
                anchors.fill: parent
                model: mvc_lidar_model
                delegate: mvc_lidar_delegate
                highlight: Rectangle {
                    color: "lightsteelblue"
                    width: mvc_lidar_listview.width
                    radius: 5
                }
                focus: true
                cacheBuffer: 10000
            }


    Button {
        id: settings_close_button
        objectName: "settings_close_button"
        anchors{
            left: parent.left
            top: parent.top
            topMargin: parent.height/2 + 25
        }
        icon.name: "settings"
        icon.source: "./images/close-window-128.gif"
        icon.color: "#620b66"
        icon.width: 50
        icon.height: 50
        signal settingsClose(string obj);
        /*
        OpacityAnimator {
               id: in_animator
               target: settings_open_button;
               from: 0;
               to: 1;
               duration: 500
               running: false
           }
        */
        onClicked: {
                    //in_animator.running = true;
                    stackview_settings.pop();
                    //animation_right_reverse.running = true;
                    //animation_bottom_reverse.running = true;
                    settings_close_button.settingsClose("Show settings panel");
                    }
        background: Rectangle {
            id: power_button_bg
            color: parent.down? "#b1b1b1" : "#00fbfbfb"
            radius: 10
            border.color: "#3afbfbfb"
        }
    }
}
        Component.onCompleted: {
            var db = LocalStorage.openDatabaseSync("ScannerSettingsDB", "1.0", "Your QML SQL", 1000000);
            db.transaction(
                function(tx) {
                    var rs = tx.executeSql('SELECT * FROM BooleanSettings where name = "Display Reconstruction"');
                    mvc_lidar_model.display_reconstruction_value = rs.rows.item(0).value;
                }
            )
        }
}
