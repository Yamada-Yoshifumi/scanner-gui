import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.LocalStorage 2.15
import "model_handler.js" as JS

Rectangle {
    id: settings_general
    width: parent.width
    height: parent.height
    color: "#442e5d"
    border.color: "#442e5d"
    property string textcolor: "#ffffff"
    signal daylightModeChanged(string str)
    function daylightModeChange(){
                var db = LocalStorage.openDatabaseSync("ScannerSettingsDB", "1.0", "Your QML SQL", 1000000);

                db.transaction(
                    function(tx) {
                        var rs = tx.executeSql('SELECT * FROM BooleanSettings where name = "Daylight Mode"');
                        var daylight_mode = rs.rows.item(0).value;
                        if (daylight_mode){
                            color = "#f5f55b";
                            border.color = "#f5f55b";
                            settings_general_header.color = "black";
                            general_listview.color = "#f7f78d";
                            textcolor = "black";
                        }
                        else{
                            color = "#442e5d";
                            border.color = "#442e5d";
                            settings_general_header.color = "#ffffff";
                            general_listview.color = "#442e5d";
                            textcolor = "#ffffff";
                        }
                    }
                )
        }
    Text {
        id: settings_general_header
        x: parent.x
        y: parent.y
        width: parent.width
        height: parent.width / 5

        color: settings_general.textcolor
        text: "General"
        horizontalAlignment: Text.AlignHCenter
        font.pointSize: 100 * parent.width/ 2560
        font.styleName: "Bold"

    }

    ListModel {
            id: mvc_general_model
            property int daylight_mode: 0
            /*
            ListElement {
                name: "Display Reconstruction"
                value: mvc_general_model.display_reconstruction_value
                cache: true
            }
            */
            property bool completed: false
            Component.onCompleted: {
                append({name: "Daylight Mode", value: mvc_general_model.daylight_mode});
                completed = true;
            }

            // 2. Update the list model:
            onDaylight_modeChanged: {
                if(completed) setProperty(0, "value", mvc_general_model.daylight_mode);
            }
        }

        // 2. Delegate - this describes how to handle each row of data
        Component {
            id: mvc_general_delegate
            Row {
                Text {
                    id: mvc_general_level_1
                    text: name
                    width: settings_general_header.width/2
                    height: settings_general_header.width/ 5
                    color: settings_general.textcolor
                    horizontalAlignment: Text.AlignHCenter
                    font.pointSize: 100 * parent.width/ 2560
                    MouseArea {
                        anchors.fill: parent
                        onClicked: {
                            mvc_general_listview.currentIndex = index
                            console.debug("Clicked on age")
                            JS.doSomething()  // Javascript from file
                        }
                    }
                }
                Switch {
                    position: value
                    checked: value
                    width: settings_general_header.width/2
                    height: settings_general_header.width/ 5

                    property bool init: false
                    MouseArea {
                        anchors.fill: parent
                        onClicked: {
                            value = value === 0? 1: 0;
                            var db = LocalStorage.openDatabaseSync("ScannerSettingsDB", "1.0", "Your QML SQL", 1000000);
                            db.transaction(
                                function(tx) {
                                    tx.executeSql('UPDATE BooleanSettings SET value = ? WHERE name="Daylight Mode"', value);
                                    settings_general.daylightModeChanged("daylight mode toggled");
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
            id: general_listview
            color: "#442e5d"

            anchors.top: settings_general_header.bottom
            anchors.bottom: settings_general.bottom
            width: settings_general.width
            ListView {
                id: mvc_general_listview
                anchors.fill: parent
                model: mvc_general_model
                delegate: mvc_general_delegate
                highlight: Rectangle {
                    color: "lightsteelblue"
                    width: mvc_general_listview.width
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
                    stackview_settings.pagePopedPushed("reload color");
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
            settings_general.daylightModeChanged.connect(settings_general.daylightModeChange);
            var db = LocalStorage.openDatabaseSync("ScannerSettingsDB", "1.0", "Your QML SQL", 1000000);
            db.transaction(
                function(tx) {
                    var rs = tx.executeSql('SELECT * FROM BooleanSettings where name = "Daylight Mode"');
                    mvc_general_model.daylight_mode = rs.rows.item(0).value;
                    rs = tx.executeSql('SELECT * FROM BooleanSettings where name = "Daylight Mode"');
                    var daylight_mode = rs.rows.item(0).value;
                    if (daylight_mode){
                        color = "#f5f55b";
                        border.color = "#f5f55b";
                        settings_general_header.color = "black";
                        general_listview.color = "#f7f78d";
                        textcolor = "black";
                    }
                    else{
                        color = "#442e5d";
                        border.color = "#442e5d";
                        settings_general_header.color = "#ffffff";
                        general_listview.color = "#442e5d";
                        textcolor = "#ffffff";
                    }
                }
            )
        }
}
