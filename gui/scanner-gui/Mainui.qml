
import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.3
import QtQuick.Controls.Universal 2.15
import QtQuick.Window 2.15
import QtMultimedia 5.15
import QtGraphicalEffects 1.0

Rectangle{

id: ui_page
width: parent.width
height: parent.height
x: 0
y: 0
color: "#442e5d"

Rectangle{
    id: rviz_window
        x: 0
        y: 0
        width: parent.width * 2 / 3
        height: parent.height * 2 / 3
        color: "#00000000"
        border.color: "#00000000"
        /*
        NumberAnimation {
                    id: fullscreen_animation_x
                    target: rviz_window
                    property: "width"
                    from: rviz_window.width
                    to: ui_page.width
                    duration: 300
                    easing.type: Easing.InExpo
                }
        NumberAnimation {
                    id: fullscreen_animation_y
                    target: rviz_window
                    property: "height"
                    from: rviz_window.height
                    to: ui_page.height
                    duration: 300
                    easing.type: Easing.InExpo
                }
        NumberAnimation {
                    id: exit_fullscreen_animation_x
                    target: rviz_window
                    property: "width"
                    from: ui_page.width
                    to: ui_page.width * 2 / 3
                    duration: 300
                    easing.type: Easing.InExpo
                }
        NumberAnimation {
                    id: exit_fullscreen_animation_y
                    target: rviz_window
                    property: "height"
                    from: ui_page.height
                    to: ui_page.height * 2 / 3
                    duration: 300
                    easing.type: Easing.InExpo
                }
                */
        /*
        Button {
                    id: fullscreen_toggle_button
                    anchors{
                        right: parent.right
                        bottom: parent.bottom
                    }
                    icon.name: "fullscreen_toggle"
                    icon.source: "./images/fullscreen.svg"
                    icon.color: "#620b66"
                    icon.width: 64* root.width/ 2560
                    icon.height: 64* root.height/ 1600


                    onClicked: {
                        fullscreen_animation_x.running = true;
                        fullscreen_animation_y.running = true;
                    }

                    background: Rectangle {
                        id: fullscreen_button_bg
                        color: parent.down? "#b1b1b1" : "#00fbfbfb"
                        radius: 10
                        border.color: "#3afbfbfb"

                    }
        }
        */
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
        color: "#343434"
        border.color: "#00000000"
        radius: 10
        LinearGradient {
            id: linearGradient
                anchors.fill: parent
                source: parent
                start: Qt.point(0, 0)
                end: Qt.point(100 * ui_page.width/ 2560, 20 * ui_page.width/ 2560)
                gradient: Gradient {
                    GradientStop { position: 0.0; color: "white" }
                    GradientStop { position: 1.0; color: "#343434" }
                }
            }
        Text{
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
                    end: Qt.point(50 * ui_page.width/ 2560, 100 * ui_page.width/ 2560)
                    gradient: Gradient {
                        GradientStop { position: 0.0; color: "white" }
                        GradientStop { position: 1.0; color: "#343434" }
                    }
                }

        ColumnLayout{

            id: lidar_panel
            width: parent.width
            height: parent.height
            Layout.maximumHeight: parent.height
            spacing: 5

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

            IndicatorLED{
                Layout.fillWidth: true
                Layout.preferredHeight: parent.height*0.30
                Layout.maximumHeight: parent.height*0.30
                id: lidar_status
                objectName: "lidar_status"
                colour: "red"
                onPaint: {
                    squircle();
                    lidar_status_text.text = colour == "red"? "offline" : "online";
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
                    x: parent.width * 2 / 3
                    verticalAlignment: Text.AlignVCenter
                    font.pointSize: 15* ui_page.width/ 2560
                    color: "#948e8e"
                }

                color: "#343434"
                border.color: "#343434"
            }

        }
        Image { objectName: "lidar_status_picture"; source: "./images/lidar.png"; x:0; y:parent.height/2; width:parent.width/2; height:parent.height/2; opacity: 0.3 }
        /*<a href="https://www.flaticon.com/free-icons/radar" title="radar icons">Radar icons created by Icongeek26 - Flaticon</a>*/
        }
        Rectangle{
            radius: 10
            Layout.preferredHeight: parent.height * 19 / 20
            Layout.preferredWidth:  parent.width / 4.1
            LinearGradient {
                    anchors.fill: parent
                    source: parent
                    start: Qt.point(0, 0)
                    end: Qt.point(50 * ui_page.width/ 2560, 100 * ui_page.width/ 2560)
                    gradient: Gradient {
                        GradientStop { position: 0.0; color: "white" }
                        GradientStop { position: 1.0; color: "#343434" }
                    }
                }
        ColumnLayout{

            id: imu_panel
            width: parent.width
            height: parent.height
            Layout.maximumHeight: parent.height
            spacing: 5

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

            IndicatorLED{
                Layout.fillWidth: true
                Layout.preferredHeight: parent.height*0.30
                id: imu_status
                objectName: "imu_status"
                colour: "red"
                onPaint: {
                    squircle();
                    imu_status_text.text = colour == "red"? "offline" : "online";
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

                color: "#343434"
                border.color: "#343434"
            }

        }
        Image {objectName: "gyro_status_picture"; source: "./images/gyro.png"; x:0; y:parent.height/2; width:parent.width/2; height:parent.height/2; opacity: 0.3 }
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
                    end: Qt.point(50 * ui_page.width/ 2560, 100 * ui_page.width/ 2560)
                    gradient: Gradient {
                        GradientStop { position: 0.0; color: "white" }
                        GradientStop { position: 1.0; color: "#343434" }
                    }
                }

        ColumnLayout{

            id: camera_panel
            width: parent.width
            height: parent.height
            Layout.maximumHeight: parent.height
            spacing: 5

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: parent.height*0.30
                radius: 10
                color: "#00000000"
                border.color: "#00000000"

                LinearGradient {
                        anchors.fill: parent
                        source: parent
                        start: Qt.point(0, 0)
                        end: Qt.point(50 * ui_page.width/ 2560, 50 * ui_page.width/ 2560)
                        gradient: Gradient {
                            GradientStop { position: 0.0; color: "white" }
                            GradientStop { position: 1.0; color: "#343434" }
                        }
                    }

                Text {
                    anchors.fill: parent
                    text: "Camera"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    font.pointSize: 30* ui_page.width / 2560
                    color: "#d4d4d4"
                }

            }

            IndicatorLED{
                Layout.fillWidth: true
                Layout.preferredHeight: parent.height*0.30
                id: camera_status
                objectName: "camera_status"
                colour: "red"
                onPaint: {
                    squircle();
                    camera_status_text.text = colour == "red"? "offline" : "online";
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

                color: "#343434"
                border.color: "#343434"
            }

        }
        Image { objectName: "camera_status_picture"; source: "./images/camera.png"; x:0; y:parent.height/2; width:parent.width/2; height:parent.height/2; opacity: 0.3 }
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
                end: Qt.point(50 * ui_page.width/ 2560, 100 * ui_page.width/ 2560)
                gradient: Gradient {
                    GradientStop { position: 0.0; color: "white" }
                    GradientStop { position: 1.0; color: "#343434" }
                }
            }

        Button {
            id: power_button
            objectName: "power_button"
            icon.name: "power-button"

            icon.source: "./images/power-button.png"
            icon.color: "#620b66"
            icon.width: 180* ui_page.width/ 2560
            icon.height: 180* ui_page.height/ 1600
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter
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

    Item {
        id: videoOutput
        x: rviz_window.width + 5
        y: 5
        width: parent.width - x - 5
        height: (parent.height - status_section.height) - 5
        ColumnLayout
        {
            spacing: 5
            width: parent.width
            height: parent.height
            Layout.maximumHeight: parent.height
            ComboBox {
                Layout.maximumWidth: parent.width

                currentIndex: 2
                model: [ "Banana", "Apple", "Coconut" ]
                width: 200
                onCurrentIndexChanged: console.debug(cbItems.get(currentIndex).text + ", " + cbItems.get(currentIndex).color)
            }
            Image{
                        id: opencvImage
                        Layout.preferredHeight: parent.width * 1600 / 2560
                        Layout.maximumWidth: parent.width
                        Layout.preferredWidth: parent.width
                        objectName: "opencvImage"
                        property bool counter: false
                        visible: true
                        source: "./images/IMA_BLO_CORP_lidar-photogrammetry_lidar_pointcloud.jpg"
                        asynchronous: false
                        cache: false
            }
        }
    }

    Rectangle {
        id: log_terminal
        x: rviz_window.width + 5
        y: rviz_window.height + 5
        width: parent.width - x
        height: status_power.height + status_topbar.height
        color: "#000000"
        border.color: "#ffffff"
        Text{
            text: "This is going to be a ros log terminal"
            font.styleName: "Regular"
            width: parent.width
            height: parent.height
            color: "#ffffff"
        }
    }
}
