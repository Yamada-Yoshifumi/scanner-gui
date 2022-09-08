cmake_minimum_required(VERSION 3.16)
project(gui)

add_compile_options(-std=c++17)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
list(APPEND CMAKE_PREFIX_PATH "~/Qt/5.15.2/gcc_64")

find_package(catkin COMPONENTS rviz roscpp std_msgs sensor_msgs cv_bridge image_transport velodyne_description ros_srv REQUIRED)
catkin_package( CATKIN_DEPENDS std_msgs sensor_msgs cv_bridge image_transport roscpp velodyne_description ros_srv)
find_package(OpenCV REQUIRED PATHS ~/opencv-4.5.0/build NO_DEFAULT_PATH) # manual specification of the OpenCVConfig.cmake path is necessary since otherwise, cmake assigns /opt/local a higher priority and ocv2.4 will be used
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    include
)
link_directories(${catkin_LIBRARY_DIRS})

find_package(Qt5 REQUIRED Qml Quick Gui Core Widgets QuickWidgets)

#qt6_add_qml_module(gui
#    URI gui
#    VERSION 1.0
#   QML_FILES
#        scanner-gui/App.qml
#        scanner-gui/Bootpage.qml
#        scanner-gui/Mainui.qml
#        scanner-gui/StackViewPage.qml
#        scanner-gui/IndicatorLED.qml
#        scanner-gui/StackViewSettings.qml
#        scanner-gui/SettingsUI.qml
#        scanner-gui/SettingsInit.qml
#    RESOURCES
#        scanner-gui/fonts/fonts.txt
#        scanner-gui/images/close-window-128.gif
#        scanner-gui/images/IMA_BLO_CORP_lidar-photogrammetry_lidar_pointcloud.jpg
#        scanner-gui/images/power-button.png
#        scanner-gui/images/ros_rviz.png
#        scanner-gui/images/settings-17-128.gif
#        scanner-gui/images/fullscreen.svg
#        scanner-gui/images/exit_fullscreen.png
#)

#target_compile_definitions(scanner_gui
#    PRIVATE $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:QT_QML_DEBUG>)
add_executable(qt_gui
    src/main.cpp
    src/mainwindow.ui
    include/mainwindow.h
    src/mainwindow.cpp
    include/myviz.h
    src/myviz.cpp
    include/roshandler.h
    src/roshandler.cpp
    scanner-gui/Resource.qrc
    src/videostreamer.cpp
    include/videostreamer.h
    src/opencvimageprovider.cpp
    include/opencvimageprovider.h
    src/touchpad.cpp
    include/touchpad.h
    src/logterminal.cpp
    include/logterminal.h
    src/roundedbutton.cpp
    include/roundedbutton.h
    include/animatedgridlayout.h
    src/animatedgridlayout.cpp
)

#set_target_properties(scanner_gui PROPERTIES
#    WIN32_EXECUTABLE TRUE
#    MACOSX_BUNDLE TRUE
#)

#target_compile_definitions(scanner_gui
#    PRIVATE $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:QT_QML_DEBUG>)
target_link_libraries(qt_gui sqlite3 Qt5::Qml Qt5::Widgets Qt5::Gui Qt5::Quick Qt5::QuickWidgets ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_qml_module(MyRviz
#    URI MyRviz
#    VERSION 1.0
#    SOURCES myviz.h myviz.cpp
#)

#target_link_libraries(MyRviz PRIVATE Qt6::Core Qt6::Qml Qt6::Widgets Qt6::Gui Qt6::Quick ${catkin_LIBRARIES})

#target_include_directories(MyRvizplugin
#     PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
#)

#target_link_libraries(scanner_gui PRIVATE MyRviz)
#install(TARGETS MyRviz
#    RUNTIME DESTINATION "${INSTALL_EXAMPLEDIR}"
#    BUNDLE DESTINATION "${INSTALL_EXAMPLEDIR}"
#    LIBRARY DESTINATION "${INSTALL_EXAMPLEDIR}"
#)
