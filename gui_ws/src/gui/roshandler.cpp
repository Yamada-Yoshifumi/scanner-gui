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
    //ros_timer = new QTimer();
    //ros_timer->start(200);
    n_.reset(new ros::NodeHandle("ros_handler"));


    velodyneSwitchClient = n_->serviceClient<ros_srv::VelodyneSwitch>("/hardware_signal/velodyneSwitch");
    //std::string velodyne_points;
    //n_->param<std::string>("velodyne_points", velodyne_points, "/velodyne_points");
    //velodynesub = n_->subscribe<sensor_msgs::PointCloud2>(velodyne_points, 1, &ROSHandler::updateVelodyneStatus, this);
    //connect(velodyne_timer, SIGNAL(timeout()), this, SLOT(resetVelodyneStatus()));
}
/*
void ROSHandler::spinOnce(){
    if(ros::ok()){
        ros::spinOnce();
    }
    else
        QApplication::quit();
}
*/
bool ROSHandler::systemPowerToggle(){
    bool success = velodyneOn();
    return success;
}

bool ROSHandler::velodyneOn(){
    velodynePowerSrv.request.command = velodyneCmd;

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

/*
void ROSHandler::updateVelodyneStatus(const sensor_msgs::PointCloud2ConstPtr &msg){
    velodyne_timer->start(1000);
    velodyneCmd = 1;
}

void ROSHandler::resetVelodyneStatus(){
    velodyneCmd = 0;
}
*/
