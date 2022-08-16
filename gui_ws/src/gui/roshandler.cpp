#include <ros/ros.h>
#include "qapplication.h"
#include "roshandler.h"
#include "ros_srv/VelodyneSwitch.h"
#include <QQuickView>
#include <QQuickItem>
#include <QtQml>
#include <sensor_msgs/PointCloud2.h>

ROSHandler::ROSHandler()

{
    //velodyne_timer = new QTimer();
    //velodyne_timer->start(1000);
    ros_timer = new QTimer();
    ros_timer->start(200);
    n_.reset(new ros::NodeHandle("ros_handler"));


    velodyneSwitchClient = n_->serviceClient<ros_srv::VelodyneSwitch>("/hardware_signal/velodyneSwitch");
    //std::string velodyne_points;
    //n_->param<std::string>("velodyne_points", velodyne_points, "/velodyne_points");
    //velodynesub = n_->subscribe<sensor_msgs::PointCloud2>(velodyne_points, 1, &ROSHandler::updateVelodyneStatus, this);
    //connect(velodyne_timer, SIGNAL(timeout()), this, SLOT(resetVelodyneStatus()));
    connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
}

void ROSHandler::spinOnce(){
    if(ros::ok()){
        ros::spinOnce();
    }
    else
        QApplication::quit();
}

void ROSHandler::systemPowerToggle(){
    if (velodyneCmd == 1)
        velodyneOn();
    else
        velodyneOff();
}

void ROSHandler::velodyneOn(){
    velodynePowerSrv.request.command = 1;

    velodyneSwitchClient.call(velodynePowerSrv);
}

void ROSHandler::velodyneOff(){
    velodynePowerSrv.request.command = 0;

    velodyneSwitchClient.call(velodynePowerSrv);
}


/*
void ROSHandler::updateVelodyneStatus(const sensor_msgs::PointCloud2ConstPtr &msg){
    velodyne_timer->start(1000);
    velodyneCmd = 1;
}

void ROSHandler::resetVelodyneStatus(){
    velodyneCmd = 0;
}
*/
