import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.LocalStorage 2.15

Rectangle {
    id: settings_general
    width: parent.width
    height: parent.height
    color: "#442e5d"
    border.color: "#442e5d"
    property string textcolor: "#ffffff"
    property var db
    signal daylightModeChanged(string str)
    function daylightModeChange(){
                db.transaction(
                    function(tx) {
                        var rs = tx.executeSql('SELECT * FROM BooleanSettings where name = "Daylight Mode"');
                        var daylight_mode = rs.rows.item(0).value;
                        if (daylight_mode){
                            color = "#d2f2fc";
                            border.color = "#d2f2fc";
                            settings_general_header.color = "black";
                            general_listview.color = "#d2f2fc";
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
        x: parent.x + 50
        y: parent.y
        width: parent.width - 50
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
            property int debug_mode: 0
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
                append({name: "Debug Mode", value: mvc_general_model.debug_mode});
                completed = true;
            }

            // 2. Update the list model:
            onDaylight_modeChanged: {
                if(completed) setProperty(0, "value", mvc_general_model.daylight_mode);
            }
            onDebug_modeChanged: {
                if(completed) setProperty(1, "value", mvc_general_model.debug_mode);
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
                    verticalAlignment: Text.AlignVCenter
                    font.pointSize: 100 * parent.width/ 2560
                    MouseArea {
                        anchors.fill: parent
                        onClicked: {
                            mvc_general_listview.currentIndex = index
                            console.debug("Clicked on age")
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
                                    if(name === "Daylight Mode"){
                                        tx.executeSql('UPDATE BooleanSettings SET value = ? WHERE name="Daylight Mode"', value);
                                        settings_general.daylightModeChanged("daylight mode toggled");
                                    }
                                    else if(name === "Debug Mode"){
                                        tx.executeSql('UPDATE BooleanSettings SET value = ? WHERE name="Debug Mode"', value);
                                    }
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
            anchors.left: settings_general.left
            anchors.leftMargin: 50
            width: settings_general.width - 50
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
                    stackview_settings.pagePopedPushed("reload color");
                    }
        background: Rectangle {
            id: power_button_bg
            color: parent.down? "#b1b1b1" : "#00fbfbfb"
            radius: 10
        }
    }

        Component.onCompleted: {
            settings_general.daylightModeChanged.connect(settings_general.daylightModeChange);
            db = LocalStorage.openDatabaseSync("ScannerSettingsDB", "1.0", "Your QML SQL", 1000000);
            db.transaction(
                function(tx) {
                    var rs = tx.executeSql('SELECT * FROM BooleanSettings where name = "Daylight Mode"');
                    mvc_general_model.daylight_mode = rs.rows.item(0).value;
                    var daylight_mode = rs.rows.item(0).value;
                    if (daylight_mode){
                        color = "#d2f2fc";
                        border.color = "#d2f2fc";
                        general_listview.color = "#d2f2fc";
                        textcolor = "black";
                    }
                    else{
                        color = "#442e5d";
                        border.color = "#442e5d";
                        general_listview.color = "#442e5d";
                        textcolor = "#ffffff";
                    }
                    rs = tx.executeSql('SELECT * FROM BooleanSettings where name = "Debug Mode"');
                    mvc_general_model.debug_mode = rs.rows.item(0).value;
                }
            )
        }
}
