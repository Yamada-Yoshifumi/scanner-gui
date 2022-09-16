cmake_minimum_required(VERSION 3.16)
project(ros_srv)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(ros_srv
    "srv/VelodyneSwitch.srv"
    "srv/ImuSwitch.srv"
    "srv/CameraExposure.srv"
    "srv/Reconstruction.srv"
    "srv/ScanToggle.srv"
    "srv/RecordToggle.srv"
    "srv/SLAMModeSwitch.srv"
    DEPENDENCIES geometry_msgs std_msgs builtin_interfaces
)

ament_package()
