#ifndef ROSHANDLER_H
#define ROSHANDLER_H
#include <ros/ros.h>
#include <QQuickView>
#include <QMainWindow>
#include "ros_srv/VelodyneSwitch.h"

class ROSHandler
{
    private:

        ros::ServiceClient velodyneSwitchClient;
        ros_srv::VelodyneSwitch velodynePowerSrv;

    public:
        ROSHandler(ros::NodeHandle &n);
        bool systemPowerOn();
        bool systemPowerOff();
        bool velodyneOn();
};

#endif // ROSHANDLER_H
