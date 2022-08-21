cmake_minimum_required(VERSION 3.0.2)
project(velodyne_description)

add_compile_options(-std=c++17)

find_package(catkin COMPONENTS rviz roscpp std_msgs ros_srv REQUIRED)
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS std_msgs roscpp rviz std_msgs ros_srv)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

install(DIRECTORY launch meshes rviz urdf world include src
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(hardware_signal
    src/main.cpp
)

target_link_libraries(hardware_signal ${catkin_LIBRARIES})
