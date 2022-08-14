#include <ros/ros.h>
#include "roshandler.h"
#include "ros_srv/VelodyneSwitch.h"
#include <QQuickView>
#include <QQuickItem>
#include <QtQml>

ROSHandler::ROSHandler(ros::NodeHandle &n)

{
    velodyneSwitchClient = n.serviceClient<ros_srv::VelodyneSwitch>("/hardware_signal/velodyneSwitch");
}

bool ROSHandler::systemPowerOff(){
    return false;
}

void ROSHandler::systemPowerOn(){
    bool success = velodyneOn();
}

bool ROSHandler::velodyneOn(){
    velodynePowerSrv.request.command = 1;

    if (velodyneSwitchClient.call(velodynePowerSrv))
    {
        return velodynePowerSrv.response.success;
    }
    else
    {
        ROS_INFO("Service Unreachable");
        return false;
    }
}
