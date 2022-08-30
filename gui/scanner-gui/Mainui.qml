
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.3
import QtQuick.Controls.Universal 2.15
import QtQuick.Window 2.15
import QtMultimedia 5.15
import QtGraphicalEffects 1.0
import QtQuick.LocalStorage 2.15

Rectangle{

id: ui_page
x: 0
y: 0
width: parent.width
height: parent.height
color: "#442e5d"

Timer {
        interval: 1000; running: true; repeat: true
        onTriggered: {
            var db = LocalStorage.openDatabaseSync("ScannerSettingsDB", "1.0", "Your QML SQL", 1000000);

            db.transaction(
                function(tx) {
                    var rs = tx.executeSql('SELECT * FROM BooleanSettings where name = "Daylight Mode"');
                    var daylight_mode = rs.rows.item(0).value;
                    if (daylight_mode){
                        status_topbar.startColor = "white";
                        status_topbar.stopColor = "#fafaa2";
                        status_topbar_text.color = "white";
                    }
                    else{
                        status_topbar.startColor = "#c98cf5";
                        status_topbar.stopColor = "#b452fa";
                        status_topbar_text.color = "#d4d4d4";
                    }
                }
            )
        }
    }

Rectangle{
    id: rviz_window
        x: 0
        y: 0
        width: parent.width * 2 / 3
        height: parent.height * 2 / 3
        color: "transparent"
        border.color: "#00000000"
     }

    Rectangle{
        id: status_section
        x: rviz_window.x + 5
        y: rviz_window.height + 5
        width: rviz_window.width- 10
        color: "#442e5d"
        border.color: "#442e5d"
        height: (parent.height - rviz_window.height - 10)
        radius: 10
    }

    Rectangle {
        id: status_topbar
        x: status_section.x + 5
        y: status_section.y + 5
        width: status_section.width - 10
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
                end: Qt.point(100 * ui_page.width/ 2560, 20 * ui_page.width/ 2560)
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
            radius: 10
            Layout.preferredHeight: parent.height * 19 / 20
            Layout.preferredWidth:  parent.width / 4.1
            LinearGradient {
                    anchors.fill: parent
                    source: parent
                    start: Qt.point(0, 0)
                    end: Qt.point(0, parent.height/2)
                    gradient: Gradient {
                        GradientStop { position: 0.0; color: "#b452fa" }
                        GradientStop { position: 1.0; color: "#b617cf" }
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
            radius: 10
            Layout.preferredHeight: parent.height * 19 / 20
            Layout.preferredWidth:  parent.width / 4.1
            LinearGradient {
                    anchors.fill: parent
                    source: parent
                    start: Qt.point(0, 0)
                    end: Qt.point(0, parent.height/2)
                    gradient: Gradient {
                        GradientStop { position: 0.0; color: "#b452fa" }
                        GradientStop { position: 1.0; color: "#9613ab" }
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
            radius: 10
            Layout.preferredHeight: parent.height * 19 / 20
            Layout.preferredWidth:  parent.width / 4.1
            LinearGradient {
                id: linearGradient1
                    anchors.fill: parent
                    source: parent
                    start: Qt.point(0, 0)
                    end: Qt.point(0, parent.height/2)
                    gradient: Gradient {
                        GradientStop { position: 0.0; color: "#b452fa" }
                        GradientStop { position: 1.0; color: "#710a82" }
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



        LinearGradient {
                anchors.fill: parent
                source: parent
                start: Qt.point(0, 0)
                end: Qt.point(0, parent.height/2)
                gradient: Gradient {
                    GradientStop { position: 0.0; color: "#b452fa" }
                    GradientStop { position: 1.0; color: "#470452" }
                }
            }
        Image { source: "./images/panel_top.png"; anchors.topMargin: 0; anchors.leftMargin: 0; height: panel_top.height; width: panel_top.width; fillMode: Image.PreserveAspectFit; opacity: 1 }

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
            Button {
                id: scan_button
                objectName: "scan_button"

                Layout.preferredHeight: parent.height/3
                Layout.preferredWidth: parent.width/1.5
                Layout.maximumHeight: parent.height/3
                Layout.maximumWidth: parent.width/1.5
                anchors.rightMargin: 0
                Layout.alignment: Qt.AlignHCenter
                property bool scanning: false

                signal scanSignal(string obj)
                onClicked:{
                    scan_button.scanSignal("toggle scanning");
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
                    font.pointSize: 30* ui_page.width / 2560
                    color: "#d4d4d4"
                }

                background: Rectangle {
                                        objectName: "scan_button_bg"
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
        x: rviz_window.width + 5
        y: rviz_window.height + 5
        width: parent.width - x
        height: status_power.height + status_topbar.height
        color: "black"
        Image{
                    id: opencvImage
                    objectName: "opencv_image"
                    anchors.fill:parent
                    fillMode: Image.PreserveAspectFit
                    property bool counter: false
                    visible: true
                    source: "./images/IMA_BLO_CORP_lidar-photogrammetry_lidar_pointcloud.jpg"
                    asynchronous: false
                    cache: false
                    onSourceChanged: counter = !counter;
        }
        ComboBox {
            currentIndex: 2
            model: [ "Banana", "Apple", "Coconut" ]
            width: 200* ui_page.width/2560
            height: 80* ui_page.width/2560
            onCurrentIndexChanged: console.debug(cbItems.get(currentIndex).text + ", " + cbItems.get(currentIndex).color)
        }
    }
}
