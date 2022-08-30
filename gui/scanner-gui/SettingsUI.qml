import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.LocalStorage 2.15
import "model_handler.js" as JS

Rectangle {
    id: settings_ui
    width: parent.width
    height: parent.height
    color: "#442e5d"
    border.color: "#442e5d"
    property string textcolor: "#ffffff"
    Timer {
            interval: 100; running: true; repeat: true
            onTriggered: {
                var db = LocalStorage.openDatabaseSync("ScannerSettingsDB", "1.0", "Your QML SQL", 1000000);

                db.transaction(
                    function(tx) {
                        var rs = tx.executeSql('SELECT * FROM BooleanSettings where name = "Daylight Mode"');
                        var daylight_mode = rs.rows.item(0).value;
                        if (daylight_mode){
                            color = "#f5f55b";
                            border.color = "#f5f55b";
                            settings_header.color = "black";
                            listview.color = "#f7f78d";
                            textcolor = "black";
                        }
                        else{
                            color = "#442e5d";
                            border.color = "#442e5d";
                            settings_header.color = "#ffffff";
                            listview.color = "#442e5d";
                            textcolor = "#ffffff";
                        }
                    }
                )
            }
        }
    Text {
        id: settings_header
        x: parent.x
        y: parent.y
        width: parent.width
        height: parent.width / 5

        color: settings_ui.textcolor
        text: "Settings"
        horizontalAlignment: Text.AlignHCenter
        font.pointSize: 100 * parent.width/ 2560
        font.styleName: "Bold"

    }

    ListModel {
            id: mvc_model
            ListElement {
                name: "LiDAR"
                cache: true
            }
            ListElement {
                name: "IMU"
                cache: true
            }
            ListElement {
                name: "Camera"
                cache: true
            }
            ListElement{
                name: "General"
                cache: true
            }
        }

        // 2. Delegate - this describes how to handle each row of data
        Component {
            id: mvc_delegate
            Row {
                /*
                Text {
                    id: mvc_name
                    text: qsTr("Name: ") + name
                    width: 250
                    MouseArea {
                        anchors.fill: parent
                        onClicked: {
                            mvc_listview.currentIndex = index
                            console.debug("Clicked on name")
                        }
                    }
                }
                */
                Text {
                    id: mvc_level_0
                    text: name
                    width: settings_header.width
                    height: settings_header.width/ 5
                    color: settings_ui.textcolor
                    horizontalAlignment: Text.AlignHCenter
                    font.pointSize: 100 * parent.width/ 2560
                    MouseArea {
                        anchors.fill: parent
                        onClicked: {
                            if(index === 0){
                                stackview_settings.push("./SettingsLiDAR.qml");
                            }
                            else if(index === 1){

                            }
                            else if(index === 2){
                                stackview_settings.push("./SettingsCamera.qml");
                            }
                            else if(index === 3){
                                stackview_settings.push("./SettingsGeneral.qml");
                            }

                            mvc_listview.currentIndex = index
                            console.debug("Clicked on age")
                            JS.doSomething()  // Javascript from file
                        }
                    }
                }
            }
        }


        // 3. ListView - this displays the rows as a list.
        Rectangle {
            // Put the ListView inside a rectangle for more layout control
            id: listview
            color: "#442e5d"

            anchors.top: settings_header.bottom
            anchors.bottom: settings_ui.bottom
            width: settings_ui.width
            ListView {
                id: mvc_listview
                anchors.fill: parent
                model: mvc_model
                delegate: mvc_delegate
                highlight: Rectangle {
                    color: "lightsteelblue"
                    width: mvc_listview.width
                    radius: 5
                }
                focus: true

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
}
