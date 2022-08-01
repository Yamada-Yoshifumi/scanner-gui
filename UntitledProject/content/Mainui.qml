import QtQuick 2.15
import QtQuick.Controls 2.15
import UntitledProject
import QtQuick.Layouts
import QtQuick.Controls.Universal
import QtQuick.Window


Rectangle{

id: ui_page
width: parent.width
height: parent.height
x: 0
y: 0
color: "#1e1d1d"

    Rectangle{
        id: rviz_window
        x: 0
        y: 0
        width: parent.width * 2 / 3
        height: parent.height * 2 / 3
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
        width: rviz_window.width * 2 / 3
        color: "#343434"
        height: (parent.height - rviz_window.height)/6
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
        x: rviz_window.x
        y: status_topbar.y + status_topbar.height
        width: status_topbar.width
        height: (parent.height - rviz_window.height - status_topbar.height)
        spacing: 5

        ColumnLayout{

            id: lidar_panel
            width: parent.width / 3
            height: parent.height
            Layout.maximumHeight: parent.height
            spacing: 5

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: parent.height*0.31
                Layout.maximumHeight: parent.height*0.31
                id: lidar_status_text
                Text {
                    anchors.fill: parent
                    text: "Lidar"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    font.pointSize: 30* ui_page.width/ 2560
                    color: "#d4d4d4"
                }

                color: "#343434"
            }

            IndicatorLED{
                Layout.fillWidth: true
                Layout.preferredHeight: parent.height*0.31
                Layout.maximumHeight: parent.height*0.31
                id: lidar_status
                onPaint: squircle("green");
            }


        }
        ColumnLayout{

            id: imu_panel
            width: parent.width / 3
            height: parent.height
            Layout.maximumHeight: parent.height
            spacing: 5

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: parent.height*0.31
                id: imu_status_text
                Text {
                    anchors.fill: parent
                    text: "IMU"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    font.pointSize: 30* ui_page.width/2560
                    color: "#d4d4d4"
                }

                color: "#343434"
            }

            IndicatorLED{
                Layout.fillWidth: true
                Layout.preferredHeight: parent.height*0.31
                id: imu_status
                onPaint: squircle("green");
            }

        }
        ColumnLayout{

            id: camera_panel
            width: parent.width / 3
            height: parent.height
            Layout.maximumHeight: parent.height
            spacing: 5

            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: parent.height*0.31
                id: camera_status_text
                Text {
                    anchors.fill: parent
                    text: "Camera"
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    font.pointSize: 30* ui_page.width / 2560
                    color: "#d4d4d4"
                }

                color: "#343434"
            }

            IndicatorLED{
                Layout.fillWidth: true
                Layout.preferredHeight: parent.height*0.31
                id: camera_status
                onPaint: squircle("yellow");
            }

        }

    }

    Rectangle {
        id: imu_graphics
        x: status_topbar.x + status_topbar.width + 5
        y: status_topbar.y + 5
        width: rviz_window.width - status_topbar.width - 10
        height: parent.height - rviz_window.height - 10
        color: "#000000"
        border.color: "#ffffff"
        Text{
            text: "This is going to be an imu \n3d item"
            font.styleName: "Regular"
            width: parent.width
            height: parent.height
            color: "#ffffff"
        }
    }

    Rectangle {
        id: camera_stream
        x: rviz_window.width + 5
        y: 0
        width: parent.width - x
        height: (parent.height - 10) / 2
        color: "#000000"
        border.color: "#ffffff"
        Text{
            text: "This is going to be a camera stream"
            font.styleName: "Regular"
            width: parent.width
            height: parent.height
            color: "#ffffff"
        }
    }

    Rectangle {
        id: log_terminal
        x: rviz_window.width + 5
        y: camera_stream.height + 10
        width: parent.width - x
        height: camera_stream.height
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


