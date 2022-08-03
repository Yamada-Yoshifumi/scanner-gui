import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts
import QtQuick.Controls.Universal
import QtQuick.Window


Rectangle{

id: ui_page
width: parent.width
height: parent.height
x: 0
y: 0
color: "#423e3e"

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
            source: "./images/ros_rviz.png"
        }

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
        id: power_button_frame
        x: status_topbar.x + status_topbar.width + 5
        y: status_topbar.y + 5
        width: rviz_window.width - status_topbar.width - 10
        height: parent.height - rviz_window.height - 10
        color: "#00000000"
        border.color: "#00000000"
        Button {
            id: power_button

            icon.name: "power-button"
            icon.source: "./images/power-button.png"
            icon.color: "#620b66"
            icon.width: 128* root.width/ 2560
            icon.height: 128* root.height/ 1600
            anchors.verticalCenter: parent.verticalCenter
            anchors.horizontalCenter: parent.horizontalCenter

            onClicked: {
            }

            background: Rectangle {
                id: power_button_bg
                color: parent.down? "#b1b1b1" : "#423e3e"
                radius: 10
                border.color: "#3afbfbfb"

            }

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
    states: [
        State {
            name: "fullscreen"
            when: rviz_window.width === ui_page.width
            PropertyChanges {
                target:fullscreen_toggle_button;
                icon.source: "./images/exit_fullscreen.png"
            }
            PropertyChanges {
                target: fullscreen_toggle_button;
                onClicked: {
                    exit_fullscreen_animation_x.running = true;
                    exit_fullscreen_animation_y.running = true;
                }
            }
        },
        State {
            name: "power_on"
        }
    ]


}


