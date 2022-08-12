#include "hardwareswitch.h"
#include <ros/ros.h>
#include "ros_srv/VelodyneSwitch.h"

HardwareSwitch::HardwareSwitch()
{
    n_.reset(new ros::NodeHandle("qt_gui"));
    velodyneSwitchService = n_->advertiseService("velodyneSwitch", velodyneSwitch);
}

