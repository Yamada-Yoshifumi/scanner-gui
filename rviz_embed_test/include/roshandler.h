#ifndef ROSHANDLER_H
#define ROSHANDLER_H
#include "rclcpp/rclcpp.hpp"
#include <QQuickView>
#include <QMainWindow>
#include "ros_srv/srv/reconstruction.hpp"
#include "ros_srv/srv/camera_exposure.hpp"
#include "ros_srv/srv/scan_toggle.hpp"
#include "ros_srv/srv/record_toggle.hpp"
#include "ros_srv/srv/slam_mode_switch.hpp"
#include "ros_srv/srv/velodyne_switch.hpp"
#include "myviz/myviz.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class ROSHandler: public QObject
{
    Q_OBJECT;
    private:

        rclcpp::Client<ros_srv::srv::VelodyneSwitch>::SharedPtr velodyneSwitchClient;
        //ros_srv::srv::VelodyneSwitch velodynePowerSrv;
        rclcpp::Client<ros_srv::srv::CameraExposure>::SharedPtr cameraExposureUpdateClient;
        //ros_srv::srv::CameraExposure cameraExposureUpdateSrv;
        rclcpp::Client<ros_srv::srv::Reconstruction>::SharedPtr reconstructionUpdateClient;
        //ros_srv::srv::Reconstruction reconstructionUpdateSrv;
        rclcpp::Client<ros_srv::srv::ScanToggle>::SharedPtr scanToggleClient;
        //ros_srv::srv::ScanToggle scanToggleSrv;
        rclcpp::Client<ros_srv::srv::RecordToggle>::SharedPtr recordToggleClient;
        //ros_srv::srv::RecordToggle recordToggleSrv;
        rclcpp::Client<ros_srv::srv::SLAMModeSwitch>::SharedPtr slamModeSwitchClient;
        //ros_srv::srv::SLAMModeSwitch slamModeSwitchSrv;
        //ros::Subscriber velodynesub;
        QTimer *ros_timer;
        //QTimer *velodyne_timer;
        std::shared_ptr<rclcpp::Node> n_;

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
