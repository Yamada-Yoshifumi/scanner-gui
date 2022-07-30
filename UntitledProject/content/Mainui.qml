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

    RowLayout {

        id: status_power
        x: rviz_window.x
        y: rviz_window.height
        spacing: 0
        width: rviz_window.width
        height: root.height - rviz_window.height

        Rectangle{
            id: lidar_status
            Layout.fillHeight: true
            Layout.preferredWidth: parent.width * 0.25
            color: "blue"
            IndicatorLEDForm{
                width: parent.width
                height: parent.height
            }
        }
        Rectangle{
            id: imu_status
            Layout.fillHeight: true
            Layout.preferredWidth: parent.width * 0.25
            color: "red"
        }
        Rectangle{
            id: camera_status
            Layout.fillHeight: true
            Layout.preferredWidth: parent.width * 0.25
            color: "green"
        }
        Rectangle{
            id: power_button
            Layout.fillHeight: true
            Layout.preferredWidth: parent.width * 0.25
            color: "yellow"
        }

    }
}

    /*##^##
Designer {
    D{i:0;formeditorZoom:0.66}
}
##^##*/
