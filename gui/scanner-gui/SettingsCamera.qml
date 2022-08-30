import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.LocalStorage 2.15
import "model_handler.js" as JS

Rectangle {
    id: settings_camera
    width: parent.width
    height: parent.height
    color: "#442e5d"
    border.color: "#442e5d"

    Text {
        id: settings_camera_header
        x: parent.x
        y: parent.y
        width: parent.width
        height: parent.width / 5

        color: "#ffffff"
        text: "camera"
        horizontalAlignment: Text.AlignHCenter
        font.pointSize: 100 * parent.width/ 2560
        font.styleName: "Bold"

    }

    ListModel {
            id: mvc_camera_model
            property int exposure_time: 20
            /*
            ListElement {
                name: "Display Reconstruction"
                value: mvc_camera_model.display_reconstruction_value
                cache: true
            }
            */
            property bool completed: false
            Component.onCompleted: {
                append({name: "Exposure Time(ms)", value: mvc_camera_model.exposure_time});
                completed = true;
            }

            // 2. Update the list model:
            onExposure_timeChanged: {
                if(completed) setProperty(0, "value", mvc_camera_model.exposure_time);
            }
        }

        // 2. Delegate - this describes how to handle each row of data
        Component {
            id: mvc_camera_delegate
            Row {
                Text {
                    id: mvc_camera_level_1
                    text: name
                    width: settings_camera_header.width/2
                    height: settings_camera_header.width/ 5
                    color: "#d4d4d4"
                    horizontalAlignment: Text.AlignHCenter
                    font.pointSize: 100 * parent.width/ 2560
                    MouseArea {
                        anchors.fill: parent
                        onClicked: {
                            mvc_camera_listview.currentIndex = index
                            console.debug("Clicked on age")
                            JS.doSomething()  // Javascript from file
                        }
                    }
                }
                ComboBox {
                    currentIndex: value/10
                    width: settings_camera_header.width/2
                    height: settings_camera_header.width/ 5
                    model: [5, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100 ]
                    onCurrentIndexChanged:{
                        value = model[currentIndex];
                        var db = LocalStorage.openDatabaseSync("ScannerSettingsDB", "1.0", "Your QML SQL", 1000000);
                        db.transaction(
                            function(tx) {
                                tx.executeSql('UPDATE BooleanSettings SET value = ? WHERE name="Exposure Time(ms)"', value);
                            }
                        )
                    }
                }
            }
        }


        // 3. ListView - this displays the rows as a list.
        Rectangle {
            // Put the ListView inside a rectangle for more layout control
            color: "#442e5d"

            anchors.top: settings_camera_header.bottom
            anchors.bottom: settings_camera.bottom
            width: settings_camera.width
            ListView {
                id: mvc_camera_listview
                anchors.fill: parent
                model: mvc_camera_model
                delegate: mvc_camera_delegate
                highlight: Rectangle {
                    color: "lightsteelblue"
                    width: mvc_camera_listview.width
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
                    var rs = tx.executeSql('SELECT * FROM BooleanSettings where name = "Exposure Time(ms)"');
                    mvc_camera_model.exposure_time = rs.rows.item(0).value;
                }
            )
        }
}
