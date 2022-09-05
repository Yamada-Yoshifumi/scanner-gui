cmake_minimum_required(VERSION 3.0.2)
project(ros_srv)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    message_generation
)

#add_message_files(
#    FILES
#        GripperVelocity.msg
#        HardwareStatus.msg
#        ManipulationStatus.msg
#        NavigationStatus.msg
#        TeleopStatus.msg
#)
add_service_files(
    FILES
        VelodyneSwitch.srv
        ImuSwitch.srv
        CameraExposure.srv
        Reconstruction.srv
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs)
