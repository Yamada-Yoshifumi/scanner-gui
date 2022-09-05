#ifndef ROSHANDLER_H
#define ROSHANDLER_H
#include <ros/ros.h>
#include <QQuickView>
#include <QMainWindow>
#include "ros_srv/VelodyneSwitch.h"
#include "ros_srv/Reconstruction.h"
#include "ros_srv/CameraExposure.h"
#include "mainwindow.h"
#include "sensor_msgs/PointCloud2.h"

class ROSHandler: public QObject
{
    Q_OBJECT;
    private:

        ros::ServiceClient velodyneSwitchClient;
        ros_srv::VelodyneSwitch velodynePowerSrv;
        ros::ServiceClient cameraExposureUpdateClient;
        ros_srv::CameraExposure cameraExposureUpdateSrv;
        ros::ServiceClient reconstructionUpdateClient;
        ros_srv::Reconstruction reconstructionUpdateSrv;
        //ros::Subscriber velodynesub;
        QTimer *ros_timer;
        //QTimer *velodyne_timer;
        ros::NodeHandlePtr n_;

    public:
        ROSHandler();
        int velodyneCmd = 1;
        int imuCmd = 1;
        void velodyneOn();
        void velodyneOff();
        void cameraExposureUpdate(int database_camera_exposure_t);
        void reconstructionUpdate(int database_reconstruction);
        //void updateVelodyneStatus(const sensor_msgs::PointCloud2ConstPtr &msg);

    signals:
        void cameraExposureUpdatedSignal(QString);
        void hardwareOnSignal(QString);
        void hardwareOffSignal(QString);
        void reconstructionUpdatedSignal(QString);

    public Q_SLOTS:
        void systemPowerToggle();
        void scanToggle();
        //void resetVelodyneStatus();
        void spinOnce();

};

#endif // ROSHANDLER_H
