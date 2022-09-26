import QtQuick 2.1
import QtQuick.Controls 2.1
import QtQuick.LocalStorage 2.1
import Qt.labs.qmlmodels 1.0

Rectangle {
    id: settings_ui
    width: parent.width
    height: parent.height
    color: "#442e5d"
    border.color: "#442e5d"
    property string textcolor: "#ffffff"
    property var db
    function daylightModeChange(){

        db.transaction(
            function(tx) {
                var rs = tx.executeSql('SELECT * FROM BooleanSettings where name = "Daylight Mode"');
                var daylight_mode = rs.rows.item(0).value;
                if (daylight_mode){
                    color = "#d2f2fc";
                    border.color = "#d2f2fc";
                    settings_header.color = "black";
                    listview.color = "#d2f2fc";
                    if(record_button.enabled)
                        record_button_bg.normal_color = "lightsteelblue";
                    else
                         record_button_bg.normal_color = "#b1b1b1";
                    scan_button_bg.color = "lightsteelblue";
                    textcolor = "black";
                }
                else{
                    color = "#442e5d";
                    border.color = "#442e5d";
                    settings_header.color = "#ffffff";
                    listview.color = "#442e5d";
                    if(record_button.enabled)
                        record_button_bg.normal_color = "#b452fa";
                    else
                         record_button_bg.normal_color = "#b1b1b1";
                    scan_button_bg.color = "#b452fa";
                    textcolor = "#ffffff";
                }
            }
        )
    }

    Text {
        id: settings_header
        x: parent.x + 50
        y: parent.y
        width: parent.width - 50
        height: parent.width / 5

        color: settings_ui.textcolor
        text: "Settings"
        horizontalAlignment: Text.AlignHCenter
        font.pointSize: 100 * parent.width/ 2560
        font.styleName: "Bold"

    }

    ListModel {
            id: mvc_model
            property int slam_option: 0

            property bool completed: false
            Component.onCompleted: {
                append({name: "LiDAR", type:"text", cache: true});
                append({name: "IMU", type:"text", cache: true});
                append({name: "Camera", type:"text", cache: true});
                append({name: "General", type:"text", cache: true});
                append({name: "SLAM Options", type:"combo", value: mvc_model.slam_option});
                append({name: "About", type:"text", cache: true});
                append({name: "Exit", type:"exit_button", cache: true});
                completed = true;
            }

            // 2. Update the list model:
            onSlam_optionChanged:   {
                if(completed) {
                    setProperty(4, "value", mvc_model.slam_option);
                }
            }
        }
/*
        // 2. Delegate - this describes how to handle each row of data
        Component {
            id: mvc_delegate
            Row {

                Text {
                    id: mvc_level_0
                    text: name
                    width: settings_header.width
                    height: settings_header.width/ 5
                    color: settings_ui.textcolor
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    font.pointSize: 100 * parent.width/ 2560
                    MouseArea {
                        anchors.fill: parent
                        onClicked: {
                            stackview_settings.push("./Settings"+name+".qml");

                            mvc_listview.currentIndex = index
                            console.debug("Clicked on age")
                        }
                    }
                }
            }
        }
*/

    DelegateChooser {
            id: chooser
            role: "type"    
            DelegateChoice { roleValue: "text";                 Component {
                    id: mvc_delegate
                    Row {

                        Text {
                            id: mvc_level_0
                            text: name
                            width: settings_header.width
                            height: settings_header.width/ 5
                            color: settings_ui.textcolor
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            font.pointSize: 100 * parent.width/ 2560
                            MouseArea {
                                anchors.fill: parent
                                onClicked: {
                                    stackview_settings.push("./Settings"+name+".qml");

                                    mvc_listview.currentIndex = index
                                    console.debug("Clicked on age")
                                }
                            }
                        }
                    }
                }
            }

            DelegateChoice { roleValue: "exit_button";                 Component {
                    id: mvc_delegate_exit
                    Row {

                        Text {
                            id: mvc_level_0_exit
                            text: name
                            width: settings_header.width
                            height: settings_header.width/ 5
                            color: settings_ui.textcolor
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            font.pointSize: 100 * parent.width/ 2560
                            MouseArea {
                                anchors.fill: parent
                                onClicked: {
                                    settings_toggle_button.exit("exit");


                                    mvc_listview.currentIndex = index
                                    console.debug("Clicked on age")
                                }
                            }
                        }
                    }
                }
            }

            DelegateChoice { roleValue: "combo";             Component {
                    id: mvc_delegate_combobox
                    Row {
                        Text {
                            id: mvc_combobox_level_1
                            text: name
                            width: settings_header.width/2
                            height: settings_header.width/ 5
                            color: settings_ui.textcolor
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                            font.pointSize: 100 * parent.width/ 2560
                            MouseArea {
                                anchors.fill: parent
                                onClicked: {
                                    mvc_listview.currentIndex = index
                                    console.debug("Clicked on age")
                                }
                            }
                        }
                        ComboBox {
                            currentIndex: value
                            width: settings_header.width/2
                            height: settings_header.width/ 5
                            model: [1,2,3]

                            onCurrentIndexChanged:{
                                value = currentIndex;
                                var db = LocalStorage.openDatabaseSync("ScannerSettingsDB", "1.0", "Your QML SQL", 1000000);
                                db.transaction(
                                    function(tx) {
                                        tx.executeSql('UPDATE BooleanSettings SET value = ? WHERE name="SLAM Options"', currentIndex);
                                    }
                                )
                            }
                        }
                    }
                } }
        }


        // 3. ListView - this displays the rows as a list.
        Rectangle {
            // Put the ListView inside a rectangle for more layout control
            id: listview

            anchors.top: settings_header.bottom
            anchors.bottom: settings_ui.bottom
            anchors.bottomMargin: settings_ui.height/5
            anchors.left: settings_ui.left
            anchors.leftMargin: 50
            width: settings_ui.width - 50
            ListView {
                id: mvc_listview
                anchors.fill: parent
                model: mvc_model
                delegate: chooser
                highlight: Rectangle {
                    color: "lightsteelblue"
                    width: mvc_listview.width
                    radius: 5
                }
                focus: true
            }
        }


    Button {
        id: settings_toggle_button
        objectName: "settings_toggle_button"
        anchors{
            left: parent.left
            bottom: parent.bottom
            bottomMargin: parent.height/2 - height/2
        }
        property string source: "./images/back_arrow.png"
        icon.name: "settings"
        icon.source: source
        icon.color: "white"
        icon.width: 40
        icon.height: 80
        signal exit(string obj)
        signal settingsToggle(string obj);
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
                    //stackview_settings.pop();
                    //animation_right_reverse.running = true;
                    //animation_bottom_reverse.running = true;
                    settings_toggle_button.settingsToggle("Toggle settings panel");
                    if(source== "./images/forward_arrow.png")
                        source = "./images/back_arrow.png";
                    else
                        source = "./images/forward_arrow.png";
        }
        background: Rectangle {
            id: power_button_bg
            color: parent.down? "#b1b1b1" : "#00fbfbfb"
            radius: 10
        }
    }

    Button {
        id: scan_button
        objectName: "scan_button"
        property bool scanning: false
        width: parent.width/3
        height: parent.height/10
        anchors.left: parent.left
        anchors.leftMargin: ( parent.width - 50 ) / 2 + 40 - width
        anchors.top: listview.bottom
        anchors.topMargin: 10

        signal scanSignal(string obj)
        onClicked:{
            scan_button.scanSignal("toggle scanning");
            if(scanning){
                record_button.enabled = true;
                record_button_bg.normal_color = scan_button_bg.normal_color;
            }
            else{
                record_button.enabled = false;
                record_button_bg.normal_color = "#b1b1b1"
            }
        }

        onScanningChanged: {
            if(scanning){
                scanning_button_text.text = "Stop Scanning";
            }
            else{
                scanning_button_text.text = "Start Scanning";
            }
        }

        Text {
            anchors.fill: parent
            id: scanning_button_text
            text: "Start Scanning"
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.pointSize: 30* settings_ui.width / 900
            color: settings_ui.textcolor
        }

        background: Rectangle {
                                id: scan_button_bg
                                objectName: "scan_button_bg"
                                radius: 10
                                property string normal_color: "#b452fa"
                                color: scan_button.down? "#b1b1b1" : normal_color
                            }
        }

    Button {
        id: record_button
        objectName: "record_button"
        property bool recording: false
        width: parent.width/3
        height: parent.height/10
        anchors.left: parent.left
        anchors.leftMargin: ( parent.width - 50 ) / 2 + 60
        anchors.top: listview.bottom
        anchors.topMargin: 10

        enabled: false

        signal recordSignal(string obj)
        onClicked:{
            record_button.recordSignal("toggle recording");
        }

        onRecordingChanged: {
            if(recording){
                record_button_text.text = "Stop Recording";
            }
            else{
                record_button_text.text = "Start Recording";
            }
        }

        Text {
            anchors.fill: parent
            id: record_button_text
            text: "Start Recording"
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.pointSize: 30* settings_ui.width / 900
            color: settings_ui.textcolor
        }

        background: Rectangle {
                                id: record_button_bg
                                objectName: "record_button_bg"
                                radius: 10
                                property string normal_color: "#b1b1b1"

                                color: record_button.down? "#b1b1b1" : normal_color
                            }
        }


        Component.onCompleted:  {
            db = LocalStorage.openDatabaseSync("ScannerSettingsDB", "1.0", "Your QML SQL", 1000000);

            db.transaction(
                function(tx) {
                    // Create the database if it doesn't already exist
                    tx.executeSql('CREATE TABLE IF NOT EXISTS BooleanSettings(name TEXT, value INTEGER, UNIQUE(name))');
                    tx.executeSql('INSERT OR IGNORE INTO BooleanSettings VALUES(?, ?)', [ "Reconstruction", 0 ]);
                    tx.executeSql('INSERT OR IGNORE INTO BooleanSettings VALUES(?, ?)', [ "Video Source", 0 ]);
                    tx.executeSql('INSERT OR IGNORE INTO BooleanSettings VALUES(?, ?)', [ "Daylight Mode", 0 ]);
                    tx.executeSql('INSERT OR IGNORE INTO BooleanSettings VALUES(?, ?)', [ "Exposure Time(ms)", 20 ]);
                    tx.executeSql('INSERT OR IGNORE INTO BooleanSettings VALUES(?, ?)', [ "Debug Mode", 0 ]);
                    tx.executeSql('INSERT OR IGNORE INTO BooleanSettings VALUES(?, ?)', [ "Default Colour", 0 ]);
                    tx.executeSql('INSERT OR IGNORE INTO BooleanSettings VALUES(?, ?)', [ "SLAM Options", 0 ]);
                }
            )

            stackview_settings.pagePopedPushed.connect(settings_ui.daylightModeChange);

            db.transaction(
                function(tx) {
                    var rs = tx.executeSql('SELECT * FROM BooleanSettings where name = "SLAM Options"');
                    mvc_model.slam_option = rs.rows.item(0).value;
                    rs = tx.executeSql('SELECT * FROM BooleanSettings where name = "Daylight Mode"');
                    var daylight_mode = rs.rows.item(0).value;
                    if (daylight_mode){
                        color = "#d2f2fc";
                        border.color = "#d2f2fc";
                        listview.color = "#d2f2fc";
                        if(record_button.enabled)
                            record_button_bg.normal_color = "lightsteelblue";
                        else
                             record_button_bg.normal_color = "#b1b1b1";
                        scan_button_bg.normal_color = "lightsteelblue";
                        textcolor = "black";
                    }
                    else{
                        color = "#442e5d";
                        border.color = "#442e5d";
                        listview.color = "#442e5d";
                        if(record_button.enabled)
                            record_button_bg.normal_color = "#b452fa";
                        else
                             record_button_bg.normal_color = "#b1b1b1";
                        scan_button_bg.normal_color = "#b452fa";
                        textcolor = "#ffffff";
                    }
                }
            )
        }
}
