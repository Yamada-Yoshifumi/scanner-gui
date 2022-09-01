import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.LocalStorage 2.15

Rectangle {
    id: init
    visible: true
    color: "#442e5d"
    border.color: "#442e5d"
    width: parent.width
    height: parent.height

    Button {
        id: settings_open_button
        objectName: "settings_open_button"
        anchors{
            left: parent.left
            top: parent.top
            topMargin: parent.height/2 + 25
        }
        icon.name: "settings"
        icon.source: "./images/settings-17-128.gif"
        icon.color: "#620b66"
        icon.width: 50
        icon.height: 50
        signal settingsInvoke(string obj)
        /*
        OpacityAnimator {
               id: out_animator
               target: settings_open_button;
               from: 1;
               to: 0;
               duration: 100
               running: false
           }
        */
        function createDb() {
                    var db = LocalStorage.openDatabaseSync("ScannerSettingsDB", "1.0", "Your QML SQL", 1000000);

                    db.transaction(
                        function(tx) {
                            // Create the database if it doesn't already exist
                            tx.executeSql('CREATE TABLE IF NOT EXISTS BooleanSettings(name TEXT, value INTEGER, UNIQUE(name))');

                            tx.executeSql('INSERT INTO BooleanSettings VALUES(?, ?)', [ "Reconstruction", 0 ]);
                            tx.executeSql('INSERT INTO BooleanSettings VALUES(?, ?)', [ "Daylight Mode", 0 ]);
                            tx.executeSql('INSERT INTO BooleanSettings VALUES(?, ?)', [ "Exposure Time(ms)", 20 ]);
                            tx.executeSql('INSERT INTO BooleanSettings VALUES(?, ?)', [ "Debug Mode", 0 ]);
                        }
                    )
                }

        onClicked: {
                    //out_animator.running = true;
                    stackview_settings.push( "SettingsUI.qml" );
                    //animation_right.running = true;
                    //animation_bottom.running = true;
                    settings_open_button.settingsInvoke("Show settings panel");
                    createDb();
                    //mainLoader.anchors.bottomMargin = settingsLoader.width * root.height/ root.width;
                 }

        background: Rectangle {
            id: power_button_bg
            color: parent.down? "#b1b1b1" : "#00fbfbfb"
            radius: 10
            border.color: "#3afbfbfb"

        }

    }
}
