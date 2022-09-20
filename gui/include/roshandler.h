#ifndef ROSHANDLER_H
#define ROSHANDLER_H
#include <ros/ros.h>
#include <QQuickView>
#include <QMainWindow>
#include "ros_srv/VelodyneSwitch.h"
#include "ros_srv/Reconstruction.h"
#include "ros_srv/CameraExposure.h"
#include "ros_srv/ScanToggle.h"
#include "ros_srv/RecordToggle.h"
#include "ros_srv/SLAMModeSwitch.h"
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
        ros::ServiceClient scanToggleClient;
        ros_srv::ScanToggle scanToggleSrv;
        ros::ServiceClient recordToggleClient;
        ros_srv::RecordToggle recordToggleSrv;
        ros::ServiceClient slamModeSwitchClient;
        ros_srv::SLAMModeSwitch slamModeSwitchSrv;
        //ros::Subscriber velodynesub;
        QTimer *ros_timer;
        //QTimer *velodyne_timer;
        ros::NodeHandlePtr n_;

    public:
        ROSHandler();
        int velodyneCmd = 1;
        int imuCmd = 1;
        int cameraCmd = 1;
        int scanCmd = 1;
        int recordCmd = 1;
        void velodyneOn();
        void velodyneOff();
        void cameraExposureUpdate(int database_camera_exposure_t);
        void reconstructionUpdate(int database_reconstruction);
        void slamModeUpdate(int database_current_slam_mode);
        //void updateVelodyneStatus(const sensor_msgs::PointCloud2ConstPtr &msg);

    signals:
        void cameraExposureUpdatedSignal(QString);
        void hardwareOnSignal(QString);
        void hardwareOffSignal(QString);
        void reconstructionUpdatedSignal(QString);
        void slamModeSwitchedSignal(QString);
        void scanToggledSignal(QString);
        void recordToggledSignal(QString);

    public Q_SLOTS:
        void systemPowerToggle();
        void scanToggle();
        void recordToggle();
        //void resetVelodyneStatus();
        void spinOnce();

};

#endif // ROSHANDLER_H