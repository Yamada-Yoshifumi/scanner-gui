cmake_minimum_required(VERSION 3.16)

project(gui)

add_compile_options(-std=c++14)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
list(APPEND CMAKE_PREFIX_PATH "~/Qt/6.3.1/gcc_64")
find_package(Qt6 6.2 COMPONENTS Quick Gui REQUIRED)
qt_add_library(STATIC)

#qt_add_executable(qt-gui
#    src/main.cpp
#)

qt6_add_qml_module(gui
    URI gui
    VERSION 1.0
    QML_FILES
        scanner-gui/App.qml
        scanner-gui/Bootpage.qml
        scanner-gui/Mainui.qml
        scanner-gui/StackViewPage.qml
        scanner-gui/IndicatorLED.qml
        scanner-gui/StackViewSettings.qml
        scanner-gui/SettingsUI.qml
        scanner-gui/SettingsInit.qml
    RESOURCES
        scanner-gui/fonts/fonts.txt
        scanner-gui/images/close-window-128.gif
        scanner-gui/images/IMA_BLO_CORP_lidar-photogrammetry_lidar_pointcloud.jpg
        scanner-gui/images/power-button.png
        scanner-gui/images/ros_rviz.png
        scanner-gui/images/settings-17-128.gif
        scanner-gui/images/fullscreen.svg
        scanner-gui/images/exit_fullscreen.png
)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

catkin_package(CATKIN_DEPENDS
    roscpp
    std_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(scanner_gui
    scanner-gui/Resource.qrc
    src/main.cpp
)

target_link_libraries(scanner_gui PRIVATE Qt6::Gui Qt6::Quick ${catkin_LIBRARIES})
