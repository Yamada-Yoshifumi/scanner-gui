#ifndef ROSHANDLER_H
#define ROSHANDLER_H
#include <ros/ros.h>
#include <QQuickView>
#include <QMainWindow>
#include "ros_srv/VelodyneSwitch.h"

class ROSHandler: public QObject
{
    Q_OBJECT;
    private:

        ros::ServiceClient velodyneSwitchClient;
        ros_srv::VelodyneSwitch velodynePowerSrv;

    public:
        ROSHandler(ros::NodeHandle &n);

        bool systemPowerOff();
        bool velodyneOn();

    public Q_SLOTS:
        void systemPowerOn();
};

#endif // ROSHANDLER_H
