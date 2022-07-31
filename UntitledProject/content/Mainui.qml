import QtQuick 2.15
import QtQuick.Controls 2.15
import UntitledProject
import QtQuick.Layouts


Rectangle{

id: ui_page
width: root.width
height: root.height
x: 0
y: 0
color: "#343434"
border.color: "#ffffff"
visible: true

    Rectangle{
        id: rviz_window
        x: 0
        y: 0
        width: root.width * 2 / 3
        height: root.height * 2 / 3
        Image {
            id: rviz_image
            x: 0
            y: 0
            width : parent.width
            height: parent.height
            source: "../src/ros_rviz.png"
        }
    }
    Rectangle {
        id: status_topbar
        x: rviz_window.x
        y: rviz_window.height
        width: rviz_window.width / 2
        color: "#343434"
        height: (root.height - rviz_window.height)/4
        Text{
            anchors.fill: parent
            horizontalAlignment: Text.AlignHCenter
            color: "#d4d4d4"
            text: "Status"
            font.pointSize: 30* root.width/ 2560
            font.styleName: "Bold"
        }
    }

    RowLayout {

        id: status_power
        x: rviz_window.x
        y: status_topbar.y + status_topbar.height
        spacing: 10
        width: rviz_window.width  * root.width/ 2560
        height: (root.height - rviz_window.height) * root.height/ 1600

        ColumnLayout{

            id: lidar_panel
            width: parent.width / 3
            height: parent.height

            Text{
                Layout.fillWidth: true
                Layout.preferredHeight: parent.height*0.3
                id: lidar_status_text
                text: "Lidar"
                horizontalAlignment: Text.AlignHCenter
                font.pointSize: 30* root.width/ 2560
                color: "#d4d4d4"
            }

            IndicatorLED{
                height: parent.height * 0.3
                width: parent.width
                id: lidar_status
                onPaint: squircle("green");
            }

            Button {
                id: reboot_lidar
                Layout.fillWidth: true
                Layout.preferredHeight: parent.height*0.3
                text: qsTr("Reconnect")
                contentItem: Text {
                    id: reboot_lidar_text
                    color: reboot_lidar.down? "#463c3c" : "#b1b1b1"
                        text: reboot_lidar.text
                        font.styleName: "Regular"
                        font.pixelSize: 30* root.width/2560
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                        elide: Text.ElideRight

                    }

                background: Rectangle {
                    id: reboot_lidar_bg
                    color: parent.down? "#b1b1b1" : "#620b66"
                    radius: 10
                    border.color: "#00ffffff"
                }

            }


        }
        ColumnLayout{

            id: imu_panel
            width: parent.width / 3
            height: parent.height

            Text{
                Layout.fillWidth: true
                Layout.preferredHeight: parent.height*0.3
                id: imu_status_text
                text: "IMU"
                horizontalAlignment: Text.AlignHCenter
                font.pointSize: 30* root.width/ 2560
                color: "#d4d4d4"
            }

            IndicatorLED{
                height: parent.height *0.3
                width: parent.width
                id: imu_status
                onPaint: squircle("green");
            }
            Button {
                id: reboot_imu
                Layout.fillWidth: true
                Layout.preferredHeight: parent.height*0.3
                text: qsTr("Reconnect")
                contentItem: Text {
                    id: reboot_imu_text
                    color: reboot_lidar.down? "#463c3c" : "#b1b1b1"
                        text: reboot_lidar.text
                        font.styleName: "Regular"
                        font.pixelSize: 30* root.width/2560
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                        elide: Text.ElideRight

                    }

                background: Rectangle {
                    id: reboot_imu_bg
                    color: parent.down? "#b1b1b1" : "#620b66"
                    radius: 10
                    border.color: "#00ffffff"
                }

            }

        }
        ColumnLayout{

            id: camera_panel
            width: parent.width / 3
            height: parent.height

            Text{
                Layout.fillWidth: true
                Layout.preferredHeight: parent.height*0.3
                id: camera_status_text
                text: "Camera"
                horizontalAlignment: Text.AlignHCenter
                font.pointSize: 30* root.width/ 2560
                color: "#d4d4d4"
            }

            IndicatorLED{
                height: parent.height *0.3
                width: parent.width
                id: camera_status
                onPaint: squircle("yellow");
            }
            Button {
                id: reboot_camera
                Layout.fillWidth: true
                Layout.preferredHeight: parent.height*0.3
                text: qsTr("Reconnect")
                contentItem: Text {
                    id: reboot_camera_text
                    color: reboot_camera.down? "#463c3c" : "#b1b1b1"
                        text: reboot_camera.text
                        font.styleName: "Regular"
                        font.pixelSize: 30* root.width/2560
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                        elide: Text.ElideRight

                    }

                background: Rectangle {
                    id: reboot_camera_bg
                    color: parent.down? "#b1b1b1" : "#620b66"
                    radius: 10
                    border.color: "#00ffffff"

                }

            }

        }
        Rectangle{
            width: parent.width / 3
            height: parent.height
            color: "#343434"
            Button {
                id: power_button
                x: camera_status.x + parent.width / 3
                y: camera_status.y + camera_status.height/2
                Layout.preferredWidth: parent.height*0.31
                Layout.preferredHeight: parent.height*0.3
                icon.name: "power"
                icon.source: "../src/power-button.png"
                icon.height: Layout.preferredHeight
                icon.width: Layout.preferredWidth


                background: Rectangle {
                    id: power_button_bg
                    color: parent.down? "#b1b1b1" : "#620b66"
                    radius: 10
                    border.color: "#00ffffff"

                }

            }
        }
        Rectangle {
            id: terminal_output
            Layout.fillWidth: true
            Layout.fillHeight: true
            Text{
                text: "boot up"
                width: parent.width
                height: parent.height
            }
        }

    }
}


