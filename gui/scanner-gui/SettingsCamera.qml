import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.LocalStorage 2.15

Rectangle {
    id: settings_camera
    width: parent.width
    height: parent.height
    color: "#442e5d"
    border.color: "#442e5d"
    property string textcolor: "#ffffff"

    Text {
        id: settings_camera_header
        x: parent.x + 50
        y: parent.y
        width: parent.width - 50
        height: parent.width / 5

        color: settings_camera.textcolor
        text: "Camera"
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
                    color: settings_camera.textcolor
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    font.pointSize: 100 * parent.width/ 2560
                    MouseArea {
                        anchors.fill: parent
                        onClicked: {
                            mvc_camera_listview.currentIndex = index
                            console.debug("Clicked on age")
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
            id: camera_listview
            color: "#442e5d"

            anchors.top: settings_camera_header.bottom
            anchors.bottom: settings_camera.bottom
            anchors.left: settings_camera.left
            anchors.leftMargin: 50
            width: settings_camera.width - 50

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

    }
    Button {
        id: settings_close_button
        objectName: "settings_close_button"
        anchors{
            left: parent.left
            bottom: parent.bottom
            bottomMargin: parent.height/2 - height/2
        }
        icon.name: "settings"
        icon.source: "./images/forward_arrow.png"
        icon.color: "white"
        icon.width: 40
        icon.height: 80
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
        }
    }

        Component.onCompleted: {
            var db = LocalStorage.openDatabaseSync("ScannerSettingsDB", "1.0", "Your QML SQL", 1000000);
            db.transaction(
                function(tx) {
                    var rs = tx.executeSql('SELECT * FROM BooleanSettings where name = "Exposure Time(ms)"');
                    mvc_camera_model.exposure_time = rs.rows.item(0).value;
                    rs = tx.executeSql('SELECT * FROM BooleanSettings where name = "Daylight Mode"');
                    var daylight_mode = rs.rows.item(0).value;
                    if (daylight_mode){
                        color = "#d2f2fc";
                        border.color = "#d2f2fc";
                        camera_listview.color = "#d2f2fc";
                        textcolor = "black";
                    }
                    else{
                        color = "#442e5d";
                        border.color = "#442e5d";
                        camera_listview.color = "#442e5d";
                        textcolor = "#ffffff";
                    }
                }
            )
        }
}
