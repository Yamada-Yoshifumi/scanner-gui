/****************************************************************************
**
** Copyright (C) 2021 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of Qt Quick Studio Components.
**
** $QT_BEGIN_LICENSE:GPL$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 3 or (at your option) any later version
** approved by the KDE Free Qt Foundation. The licenses are as published by
** the Free Software Foundation and appearing in the file LICENSE.GPL3
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-3.0.html.
**
** $QT_END_LICENSE$
**
****************************************************************************/

import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Universal
import QtQuick.Window


Rectangle{
    id: boot_window
    x: 0
    y: 0
    width: parent.width
    height: parent.height
    color: "#000000"
    border.color: "#ffffff"
    Image {
        id: lidar
        x: 0
        y: 0
        width : parent.width
        height: parent.height * 3/4
        source: "./IMA_BLO_CORP_lidar-photogrammetry_lidar_pointcloud.jpg"
    }

    Timer {
        id: timer
        interval: 1000; running: true; repeat: false
        onTriggered:{
                    if(text3.opacity === 1 && text4.opacity === 1 && text5.opacity === 1)
                        stackview.push( "Mainui.qml" )
                    else{
                        if(text3.opacity === 0) text6.opacity = 1;
                        if(text4.opacity === 0) text7.opacity = 1;
                        if(text5.opacity === 0) text8.opacity = 1;
                        reboot_button.opacity = 1;
                    }
        }
    }

    BusyIndicator {
                id: bootindicator
                running: true
                Universal.accent: Universal.Purple

                x: text2.x + text2.width + 50*parent.width/2560
                y: parent.height * 3/4
                width: 80* boot_window.width / 2560
                height: 80* boot_window.height /1600
                opacity: 1

                Behavior on opacity {
                    OpacityAnimator {
                        duration: 1000
                    }
                }

                RotationAnimator {
                    target: bootindicator
                    running: bootindicator.running
                    from: 0
                    to: 360
                    loops: Animation.Infinite
                    duration: 2000
                }
            }

    Text {
        id: text1
        x: 0
        y: 0
        width: 2269*boot_window.width/2560
        height: 120*boot_window.height/1600
        color: "#b1b1b1"
        text: qsTr("Daniel L. Lu, the copyright holder of this work used as the background here, published it under the Creative Commons Attribution 4.0 International license.")
        font.pixelSize: 30*boot_window.width/2560
    }

    Text {
        id: text2
        x: parent.width/3
        y: parent.height * 3/4
        color: "#b1b1b1"
        text: qsTr("System Booting")
        font.pixelSize: 80*boot_window.width/2560
        horizontalAlignment: Text.AlignLeft
        font.styleName: "Bold"
        opacity: 1
        Behavior on opacity { PropertyAnimation { duration: 1000 } }
    }

    Text {
        id: text3
        x: text4.x - text3.width - 100 * parent.width / 2560
        y: parent.height * 5 / 6
        color: "#b1b1b1"
        text: qsTr("Lidar Online")
        font.pixelSize: 60*boot_window.width/2560
        font.styleName: "Regular"
        opacity: 0
        Behavior on opacity { PropertyAnimation { duration: 1000 } }
    }
    Text {
        id: text6
        x: text4.x - text3.width - 100 * parent.width / 2560
        y: parent.height * 5 / 6
        color: "#b1b1b1"
        text: qsTr("Lidar Offline")
        font.pixelSize: 60*boot_window.width/2560
        font.styleName: "Regular"
        opacity: 0
        Behavior on opacity { PropertyAnimation { duration: 1000 } }
    }

    Text {
        id: text4
        x: parent.width / 3
        y: parent.height * 5 / 6
        color: "#b1b1b1"
        text: qsTr("Camera Online")
        font.pixelSize: 60*boot_window.width/2560
        font.styleName: "Regular"
        opacity: 0
        Behavior on opacity { PropertyAnimation { duration: 1000 } }
    }
    Text {
        id: text7
        x: parent.width / 3
        y: parent.height * 5 / 6
        color: "#b1b1b1"
        text: qsTr("Camera Offline")
        font.pixelSize: 60*boot_window.width/2560
        font.styleName: "Regular"
        opacity: 0
        Behavior on opacity { PropertyAnimation { duration: 1000 } }
    }

    Text {
        id: text5
        x: text4.x + text4.width + 100 * parent.width / 2560
        y: parent.height * 5 / 6
        color: "#b1b1b1"
        text: qsTr("IMU Online")
        font.pixelSize: 60*boot_window.width/2560
        font.styleName: "Regular"
        opacity: 0
        Behavior on opacity { PropertyAnimation { duration: 1000 } }
    }
    Text {
        id: text8
        x: text4.x + text4.width + 100 * parent.width / 2560
        y: parent.height * 5 / 6
        color: "#b1b1b1"
        text: qsTr("IMU Offline")
        font.pixelSize: 60*boot_window.width/2560
        font.styleName: "Regular"
        opacity: 0
        Behavior on opacity { PropertyAnimation { duration: 1000 } }
    }

    Button {
        id: reboot_button
        x: boot_window.width * 2 / 5
        y: parent.height * 9/10
        width: 200* boot_window.width / 2560
        height: 50* boot_window.width / 2560
        opacity: 0
        Behavior on opacity { PropertyAnimation { duration: 1000 } }
        text: qsTr("Reboot")
        onClicked: stackview.push( "Mainui.qml" )
        contentItem: Text {
            id: reboot_button_text
            color: reboot_button.down? "#463c3c" : "#b1b1b1"
                text: reboot_button.text
                font.styleName: "Regular"
                font.pixelSize: 30* boot_window.width/2560
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                elide: Text.ElideRight

            }

        background: Rectangle {
            id: reboot_button_bg
            width: 200* boot_window.width /2560
            height: 50* boot_window.width /2560
            color: parent.down? "#b1b1b1" : "#620b66"
            radius: 10
            border.color: "#00ffffff"
        }

    }

    states: [
        State {
            name: "boot failed"
            when: {text6.opacity === 1 || text7.opacity === 1 || text8.opacity === 1}
            PropertyChanges {target:text2; opacity: 0 }
            PropertyChanges {target:bootindicator; opacity: 0}
            PropertyChanges {
                target: text2; text: qsTr("Please reconnect your offline hardware, and press reboot")
            }
            PropertyChanges {
                target: text2; x: 0
            }

            PropertyChanges {
                target: text2; opacity:1
            }
        }
    ]

}






/*##^##
Designer {
    D{i:0;formeditorZoom:0.66}
}
##^##*/
