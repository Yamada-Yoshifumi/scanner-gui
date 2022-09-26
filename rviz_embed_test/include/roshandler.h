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

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include "rcutils/logging_macros.h"

using namespace std::chrono_literals;

class ROSHandler: public QObject
{
    Q_OBJECT;
    private:

        rclcpp::Client<ros_srv::srv::VelodyneSwitch>::SharedPtr velodyneSwitchClient;
        rclcpp::Client<ros_srv::srv::CameraExposure>::SharedPtr cameraExposureUpdateClient;
        rclcpp::Client<ros_srv::srv::Reconstruction>::SharedPtr reconstructionUpdateClient;
        rclcpp::Client<ros_srv::srv::ScanToggle>::SharedPtr scanToggleClient;
        rclcpp::Client<ros_srv::srv::RecordToggle>::SharedPtr recordToggleClient;
        rclcpp::Client<ros_srv::srv::SLAMModeSwitch>::SharedPtr slamModeSwitchClient;
        QTimer *ros_timer;
        std::shared_ptr<rclcpp::Node> n_;

        template<typename FutureT, typename WaitTimeT>
        std::future_status
        wait_for_result(
            FutureT & future,
            WaitTimeT time_to_wait)
        {
            auto end = std::chrono::steady_clock::now() + time_to_wait;
            std::chrono::milliseconds wait_period(100);
            std::future_status status = std::future_status::timeout;
            do {
                auto now = std::chrono::steady_clock::now();
                auto time_left = end - now;
                if (time_left <= std::chrono::seconds(0)) {break;}
                status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
            } while (rclcpp::ok() && status != std::future_status::ready);
            return status;
        }

        bool
        change_state(std::uint8_t transition, std::chrono::seconds time_out = 3s);
        std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;

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
        void spinOnce();
        void closeWindow();

};

#endif // ROSHANDLER_H
