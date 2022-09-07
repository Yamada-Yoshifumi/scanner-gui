import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.LocalStorage 2.15
Rectangle {
    id: settings_about
    width: parent.width
    height: parent.height
    color: "#442e5d"
    border.color: "#442e5d"
    property string textcolor: "#ffffff"

    Text {
        id: settings_about_header
        x: parent.x + 50
        y: parent.y
        width: parent.width - 50
        height: parent.width / 5

        color: settings_about.textcolor
        text: "About"
        horizontalAlignment: Text.AlignHCenter
        font.pointSize: 100 * parent.width/ 2560
        font.styleName: "Bold"

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
                var rs = tx.executeSql('SELECT * FROM BooleanSettings where name = "Daylight Mode" LIMIT 1');
                var daylight_mode = rs.rows.item(0).value;
                if (daylight_mode){
                    settings_about.color = "#d2f2fc";
                    settings_aboutborder.color = "#d2f2fc";
                    settings_abouttextcolor = "black";
                }
                else{
                    settings_aboutcolor = "#442e5d";
                    settings_aboutborder.color = "#442e5d";
                    settings_abouttextcolor = "#ffffff";
                }
            }
        )
    }
}
