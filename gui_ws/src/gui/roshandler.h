#ifndef ROSHANDLER_H
#define ROSHANDLER_H
#include <ros/ros.h>
#include <QQuickView>
#include <QMainWindow>
#include "ros_srv/VelodyneSwitch.h"
#include "mainwindow.h"
#include "sensor_msgs/PointCloud2.h"

class ROSHandler: public QObject
{
    Q_OBJECT;
    private:

        ros::ServiceClient velodyneSwitchClient;
        ros_srv::VelodyneSwitch velodynePowerSrv;
        //ros::Subscriber velodynesub;
        //QTimer *ros_timer;
        //QTimer *velodyne_timer;
        ros::NodeHandlePtr n_;

    public:
        ROSHandler();
        int velodyneCmd = 1;
        bool velodyneOn();
        //void updateVelodyneStatus(const sensor_msgs::PointCloud2ConstPtr &msg);

    public Q_SLOTS:
        bool systemPowerToggle();
        //void resetVelodyneStatus();

};

#endif // ROSHANDLER_H
