
import QtQuick 2.1
import QtQuick.Controls 2.1
import QtQuick.Layouts 1.3
import QtQuick.Window 2.1
import QtMultimedia 5.1
import QtGraphicalEffects 1.0
import QtQuick.LocalStorage 2.1

Rectangle{

id: ui_page
x: 0
y: 0
width: parent.width
height: parent.height
color: "#442e5d"
property var db

Timer {
        interval: 1000; running: true; repeat: true;
        onTriggered: {

            db.transaction(
                function(tx) {
                    var rs = tx.executeSql('SELECT * FROM BooleanSettings where name = "Daylight Mode"');
                    var daylight_mode = rs.rows.item(0).value;
                    if (daylight_mode){
                        status_topbar.startColor = "white";
                        status_topbar.stopColor = "#b5cef7";
                        status_topbar_text.color = "black";
                        status_lidar_rect.startColor = "white";
                        status_lidar_rect.stopColor = "#b5cef7";
                        status_lidar_text.color = "black";
                        status_imu_rect.startColor = "white";
                        status_imu_rect.stopColor = "#b5cef7";
                        status_imu_text.color = "black";
                        status_camera_rect.startColor = "white";
                        status_camera_rect.stopColor = "#b5cef7";
                        status_camera_text.color = "black";
                        power_button_bg.startColor = "white";
                        power_button_bg.stopColor = "#b5cef7";
                        power_button.icon.color = "#b5cef7";
                        ui_page.color = "white";
                        status_section.color = "white";
                    }
                    else{
                        status_topbar.startColor = "#c98cf5";
                        status_topbar.stopColor = "#b452fa";
                        status_topbar_text.color = "#d4d4d4";
                        status_lidar_rect.startColor = "#b452fa";
                        status_lidar_rect.stopColor = "#b617cf";
                        status_lidar_text.color = "#d4d4d4";
                        status_imu_rect.startColor = "#b452fa";
                        status_imu_rect.stopColor = "#9613ab";
                        status_imu_text.color = "#d4d4d4";
                        status_camera_rect.startColor = "#b452fa";
                        status_camera_rect.stopColor = "#710a82";
                        status_camera_text.color = "#d4d4d4";
                        power_button_bg.startColor = "#b452fa";
                        power_button_bg.stopColor = "#470452";
                        power_button.icon.color = "#620b66";
                        ui_page.color = "#442e5d";
                        status_section.color = "#442e5d";
                    }
                }
            )
        }
    }


    Rectangle{
        id: status_section
        x: 5
        y: 5
        width: parent.width*2/3- 10
        color: "#442e5d"
        height: parent.height - 10
        radius: 10
    }

    Rectangle {
        id: status_topbar
        x: status_section.x
        y: status_section.y
        width: status_section.width
        height: status_section.height/6
        border.color: "#00000000"
        radius: 10
        property string startColor: "#c98cf5"
        property string stopColor: "#b452fa"
        LinearGradient {
            id: statusTopbarLinearGradient
                anchors.fill: parent
                source: parent
                start: Qt.point(0, 0)
                end: Qt.point(0, parent.height/2)
                gradient: Gradient {
                    GradientStop { position: 0.0; color: status_topbar.startColor }
                    GradientStop { position: 1.0; color: status_topbar.stopColor }
                }
            }
        Text{
            id: status_topbar_text
            anchors.fill: parent
            horizontalAlignment: Text.AlignHCenter
            color: "#d4d4d4"
            text: "Status"
            font.pointSize: 30* ui_page.width/ 2560
            font.styleName: "Bold"
        }
    }


    RowLayout {

        id: status_power
        x: status_topbar.x
        y: status_topbar.y + status_topbar.height
        width: status_topbar.width
        height: (status_section.height - status_topbar.height - 15)
        spacing: 5

        Rectangle{
            id: status_lidar_rect
            radius: 10
            Layout.preferredHeight: parent.height * 19 / 20
            Layout.preferredWidth:  parent.width / 4.1
            property string startColor: "#b452fa"
            property string stopColor: "#b617cf"
            LinearGradient {
                    anchors.fill: parent
                    source: parent
                    start: Qt.point(0, 0)
                    end: Qt.point(0, parent.height/2)
                    gradient: Gradient {
                        GradientStop { position: 0.0; color: status_lidar_rect.startColor }
                        GradientStop { position: 1.0; color: status_lidar_rect.stopColor }
                    }
                }

        ColumnLayout{

            id: lidar_panel
            width: parent.width
            height: parent.height
            Layout.maximumHeight: parent.height
            spacing: 5

            Rectangle{
                color: "#00000000"
                width: 150* ui_page.width/ 2560
                Layout.preferredHeight: parent.height*0.1
                Image { source: "./images/panel_top.png"; anchors.fill: parent; fillMode: Image.PreserveAspectFit; opacity: 1 }
            }
            Rectangle {
                Layout.fillWidth: true
                id: lidar_rect
                color: "#00000000"
                Layout.preferredHeight: parent.height*0.30
                Layout.maximumHeight: parent.height*0.30
                radius: 10
                border.color: "#00000000"

                Text {
                    id: status_lidar_text
                    anchors.fill: parent
                    text: "Lidar"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    font.pointSize: 30* ui_page.width/2560
                    color: "#d4d4d4"
                }

            }



            Rectangle {

                Layout.fillWidth: true
                Layout.preferredHeight: parent.height*0.20
                Layout.maximumHeight: parent.height*0.20

                Text {
                    objectName: "lidar_status_text"
                    id: lidar_status_text
                    text: "online"
                    x: parent.width * 7 / 9
                    verticalAlignment: Text.AlignVCenter
                    font.pointSize: 15* ui_page.width/ 2560
                    color: "#948e8e"
                }

                color: "#00000000"
                border.color: "#00000000"
            }


        }

        IndicatorLED{
            x: parent.width/3
            y: parent.height/4
            width:parent.width
            height: parent.height*0.30
            id: lidar_status
            objectName: "lidar_status"
            colour: "red"
            onPaint: {
                squircle();
                lidar_status_text.text = colour == "red"? "offline" : "online";
            }
        }

        Image {
            id: lidar_status_picture
            objectName: "lidar_status_picture"
            source: "./images/lidar.png"
            x:0; y:parent.height/2; width:parent.height/2; height:parent.height/2;
            opacity: 0.8

            RotationAnimation on rotation {
                    id: rotateLidarPhoto;
                    loops: Animation.Infinite;
                    from: 0;
                    to: 360;
                    duration: 3000;
                    running: false;
                    paused: false;
                }

            ColorOverlay{
                anchors.fill: parent
                source: parent
                id: lidar_status_overlay
                objectName: "lidar_status_overlay"
                color: "#00000000"
                onColorChanged: {
                    //rotateLidarPhoto.running =!rotateLidarPhoto.running;

                    if(!rotateLidarPhoto.running)
                        rotateLidarPhoto.running = true;
                    else{
                        if(rotateLidarPhoto.paused)
                            rotateLidarPhoto.resume();
                        else
                            rotateLidarPhoto.pause();
                }
                }
            }

        }
            /*https://icon-icons.com/icon/radar-sweep/38952*/

    }
        Rectangle{
            id: status_imu_rect
            radius: 10
            Layout.preferredHeight: parent.height * 19 / 20
            Layout.preferredWidth:  parent.width / 4.1
            property string startColor: "#b452fa"
            property string stopColor: "#9613ab"

            LinearGradient {
                    anchors.fill: parent
                    source: parent
                    start: Qt.point(0, 0)
                    end: Qt.point(0, parent.height/2)
                    gradient: Gradient {
                        GradientStop { position: 0.0; color: status_imu_rect.startColor }
                        GradientStop { position: 1.0; color: status_imu_rect.stopColor }
                    }
                }
        ColumnLayout{

            id: imu_panel
            width: parent.width
            height: parent.height
            Layout.maximumHeight: parent.height
            spacing: 5

            Rectangle{
                color: "#00000000"
                width: 150* ui_page.width/ 2560
                Layout.preferredHeight: parent.height*0.1
                Image { source: "./images/panel_top.png"; anchors.fill: parent; fillMode: Image.PreserveAspectFit; opacity: 1 }
            }

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: parent.height*0.30
                radius: 10
                color: "#00000000"
                border.color: "#00000000"


                Text {
                    id: status_imu_text
                    anchors.fill: parent
                    text: "IMU"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    font.pointSize: 30* ui_page.width/2560
                    color: "#d4d4d4"
                }

            }



            Rectangle {

                Layout.fillWidth: true
                Layout.preferredHeight: parent.height*0.20
                Layout.maximumHeight: parent.height*0.20

                Text {
                    objectName: "imu_status_text"
                    id: imu_status_text
                    text: "online"
                    x: parent.width * 2 / 3
                    verticalAlignment: Text.AlignVCenter
                    font.pointSize: 15* ui_page.width/ 2560
                    color: "#948e8e"
                }

                color: "#00000000"
                border.color: "#00000000"
            }

        }
        IndicatorLED{
            x: parent.width/3
            y: parent.height/4
            width:parent.width
            height: parent.height*0.30
            id: imu_status
            objectName: "imu_status"
            colour: "red"
            onPaint: {
                squircle();
                imu_status_text.text = colour == "red"? "offline" : "online";
            }
        }
        Image {objectName: "gyro_status_picture"; source: "./images/gyro.png"; x:0; y:parent.height/2; width:parent.height/2; height:parent.height/2; opacity: 0.8
            ColorOverlay{
                anchors.fill: parent
                source: parent
                id: gyro_status_overlay
                objectName: "gyro_status_overlay"
                color: "#00000000"
            }
        }
        /*<a href="https://www.flaticon.com/free-icons/axis" title="axis icons">Axis icons created by Freepik - Flaticon</a>*/

        }
        Rectangle{
            id: status_camera_rect
            radius: 10
            Layout.preferredHeight: parent.height * 19 / 20
            Layout.preferredWidth:  parent.width / 4.1
            property string startColor: "#b452fa"
            property string stopColor: "#710a82"
            LinearGradient {
                id: linearGradient1
                    anchors.fill: parent
                    source: parent
                    start: Qt.point(0, 0)
                    end: Qt.point(0, parent.height/2)
                    gradient: Gradient {
                        GradientStop { position: 0.0; color: status_camera_rect.startColor }
                        GradientStop { position: 1.0; color: status_camera_rect.stopColor }
                    }
                }

        ColumnLayout{

            id: camera_panel
            width: parent.width
            height: parent.height
            Layout.maximumHeight: parent.height
            spacing: 5

            Rectangle{
                id: panel_top
                color: "#00000000"
                width: 150* ui_page.width/ 2560
                Layout.preferredHeight: parent.height*0.1
                Image { source: "./images/panel_top.png"; anchors.fill: parent; fillMode: Image.PreserveAspectFit; opacity: 1 }
            }

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: parent.height*0.30
                radius: 10
                color: "#00000000"
                border.color: "#00000000"

                Text {
                    id: status_camera_text
                    anchors.fill: parent
                    text: "Camera"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    font.pointSize: 30* ui_page.width / 2560
                    color: "#d4d4d4"
                }

            }



            Rectangle {

                Layout.fillWidth: true
                Layout.preferredHeight: parent.height*0.20
                Layout.maximumHeight: parent.height*0.20

                Text {
                    id: camera_status_text
                    objectName: "camera_status_text"
                    text: "offline"
                    x: parent.width * 2 / 3
                    verticalAlignment: Text.AlignVCenter
                    font.pointSize: 15* ui_page.width/ 2560
                    color: "#948e8e"
                }

                color: "#00000000"
                border.color: "#00000000"
            }

        }
        IndicatorLED{
            x: parent.width/3
            y: parent.height/4
            width:parent.width
            height: parent.height*0.30
            id: camera_status
            objectName: "camera_status"
            colour: "red"
            onPaint: {
                squircle();
                camera_status_text.text = colour == "red"? "offline" : "online";
            }
        }
        Image { objectName: "camera_status_picture"; source: "./images/camera.png"; x:0; y:parent.height/2; width:parent.height/2; height:parent.height/2; opacity: 0.8
            ColorOverlay{
                anchors.fill: parent
                source: parent
                id: camera_status_overlay
                objectName: "camera_status_overlay"
                color: "#00000000"
            }
        }
        /*<a href="https://www.flaticon.com/free-icons/camera" title="camera icons">Camera icons created by Good Ware - Flaticon</a>*/

    }
    Rectangle {
        id: power_button_bg
        Layout.preferredHeight: parent.height * 19 / 20
        Layout.preferredWidth:  parent.width / 4.1
        radius: 10
        property string startColor: "#b452fa"
        property string stopColor: "#470452"

        LinearGradient {
                anchors.fill: parent
                source: parent
                start: Qt.point(0, 0)
                end: Qt.point(0, parent.height/2)
                gradient: Gradient {
                    GradientStop { position: 0.0; color: power_button_bg.startColor }
                    GradientStop { position: 1.0; color: power_button_bg.stopColor}
                }
            }
        Image { source: "./images/panel_top.png"; anchors.topMargin: 5; anchors.leftMargin: 0; height: panel_top.height; width: panel_top.width; fillMode: Image.PreserveAspectFit; opacity: 1 }

        ColumnLayout{

            x: 0
            y: panel_top.height
            width: parent.width
            height: parent.height - panel_top.height
            spacing: 10

            Button {
                id: power_button
                objectName: "power_button"
                icon.name: "power-button"

                icon.source: "./images/power-button.png"
                icon.color: "#620b66"

                Layout.preferredHeight: parent.height/2.5
                Layout.preferredWidth: parent.height/2.5
                Layout.maximumHeight: parent.height/2.5
                Layout.maximumWidth: parent.width/2.5
                Layout.alignment: Qt.AlignHCenter

                icon.width: parent.height/2.5
                icon.height: parent.height/2.5

                signal powerSignal(string obj)
                onClicked:{
                    power_button.powerSignal("system power on command");
                }
                background: Rectangle {
                                        objectName: "power_button_bg"
                                        radius: 10
                                        color: power_button.down? "#b1b1b1" : "#343434"
                                        border.color: "#b1b1b1"
                                    }
                }
        }

        }

    }

    Rectangle {
        id: videoOutput
        x: parent.width*2/3 + 5
        y: 5
        width: parent.width - x
        height: status_section.height
        color: "black"
        Image{
                    id: opencvImage
                    objectName: "opencv_image"
                    anchors.fill:parent
                    fillMode: Image.PreserveAspectFit
                    property bool counter: true
                    visible: true
                    source: "./images/IMA_BLO_CORP_lidar-photogrammetry_lidar_pointcloud.jpg"
                    asynchronous: false
                    cache: false
                    onSourceChanged: counter = !counter;
        }

    }
    ComboBox {
        id: video_selection
        objectName: "video_selection"
        x:videoOutput.x
        y:videoOutput.y
        width: 300* ui_page.width/2560
        height: 80* ui_page.width/2560
        currentIndex: 0
        opacity: 0.7
        signal sourceChangeSignal(string obj)
        model: [ "Camera 1", "Camera 2"]
        onCurrentIndexChanged: {
            var db = LocalStorage.openDatabaseSync("ScannerSettingsDB", "1.0", "Your QML SQL", 1000000);
            db.transaction(
                function(tx) {
                    tx.executeSql('UPDATE BooleanSettings SET value = ? WHERE name="Video Source"', currentIndex);
                }
            )
        }
        Component.onCompleted: {
            db = LocalStorage.openDatabaseSync("ScannerSettingsDB", "1.0", "Your QML SQL", 1000000);
            db.transaction(
                function(tx) {
                    var rs = tx.executeSql('SELECT * FROM BooleanSettings where name = "Video Source" LIMIT 1');
                    currentIndex = rs.rows.item(0).value;
                }
            )
        }
    }
}
