cmake_minimum_required(VERSION 3.16)
project(rviz_embed_test)
add_compile_options(-std=c++14)


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
list(APPEND CMAKE_PREFIX_PATH "~/Qt/5.15.2/gcc_64")

find_package(ament_cmake REQUIRED COMPONENTS std_msgs sensor_msgs nav_msgs image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(OpenCV REQUIRED PATHS ~/opencv-4.5.0/build NO_DEFAULT_PATH) # manual specification of the OpenCVConfig.cmake path is necessary since otherwise, cmake assigns /opt/local a higher priority and ocv2.4 will be used
find_package(ros_srv REQUIRED)
find_package(Qt5Qml REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5QuickWidgets REQUIRED)
find_package(cv_bridge_ros2 REQUIRED)
find_package(lifecycle_msgs REQUIRED)

find_package(rmw_implementation REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  lifecycle_msgs
  rclcpp
  cv_bridge_ros2
  rclcpp_components
  rviz_common
  rviz_default_plugins
  rviz_ogre_vendor
  rviz_rendering
)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_ros2_INCLUDE_DIRS}
  ${ament_cmake_INCLUDE_DIRS}
  ${rviz_common_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5Qml_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
  ${Qt5QuickWidgets_INCLUDE_DIRS}
  ${ros_srv_INCLUDE_DIRS}
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(headers_to_moc
  include/myviz/myviz.hpp
  include/animatedgridlayout.h
  include/logterminal.h
  include/roundedbutton.h
  include/touchpad.h
  include/opencvimageprovider.h
  include/roshandler.h
  include/videostreamer.h
)

foreach(header "${headers_to_moc}")
  qt5_wrap_cpp(moc_files "${header}")
endforeach()

set(SRC_FILES
  src/myviz.cpp
  src/main.cpp
  src/animatedgridlayout.cpp
  src/logterminal.cpp
  src/roundedbutton.cpp
  src/touchpad.cpp
  src/opencvimageprovider.cpp
  src/roshandler.cpp
  src/videostreamer.cpp
  scanner-gui/Resource.qrc
  ${moc_files}
)

add_executable(${PROJECT_NAME} ${SRC_FILES})
ament_target_dependencies(${PROJECT_NAME} std_msgs lifecycle_msgs
    nav_msgs OpenCV ros_srv cv_bridge_ros2 rmw_implementation ${THIS_PACKAGE_INCLUDE_DEPENDS})

target_link_libraries(${PROJECT_NAME}
  sqlite3 Qt5::Qml Qt5::Widgets Qt5::Gui Qt5::Quick Qt5::QuickWidgets ${ament_cmake_LIBRARIES} ${OpenCV_LIBS}
  rviz_common::rviz_common
  rviz_rendering::rviz_rendering
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
)

## Install
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
