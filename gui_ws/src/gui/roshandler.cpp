#include <ros/ros.h>
#include "roshandler.h"
#include "ros_srv/VelodyneSwitch.h"

ROSHandler::ROSHandler(ros::NodeHandle &n)
{
    velodyneSwitchClient = n.serviceClient<ros_srv::VelodyneSwitch>("velodyneSwitch");
}

bool ROSHandler::systemPowerOff(){
    return false;
}

bool ROSHandler::systemPowerOn(){
    return velodyneOn();
}

bool ROSHandler::velodyneOn(){
    velodynePowerSrv.request.command = 1;
    velodyneSwitchClient.call(velodynePowerSrv);

    if (velodyneSwitchClient.call(velodynePowerSrv))
    {
        ROS_INFO("service called");
        return velodynePowerSrv.response.success;
    }
    else
    {
        ROS_INFO("service unreachable");
        return false;
    }
}
