#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "roslaunchmanager.h"
#include "ros_srv/VelodyneSwitch.h"
#include "ros_srv/CameraExposure.h"
#include <thread>

ROSLaunchManager ros_launch_manager;
pid_t hardware_launcher;

void velodyneOnThread(){
    try {
        hardware_launcher = ros_launch_manager.start(
            "velodyne_description", "simulation.launch");
    }
    catch (std::exception const &exception) {
        ROS_WARN("%s", exception.what());
        ros_launch_manager.stop(hardware_launcher, SIGINT);
    }
}

bool velodyneSwitch(ros_srv::VelodyneSwitch::Request &req, ros_srv::VelodyneSwitch::Response &res)
{
    res.success = true;

    if (req.command == 1){
        ROS_INFO("Power on request received");
        std::thread t(velodyneOnThread);
        t.detach();
    }
    else if (req.command == 0){
        ROS_INFO("Shut Down request received");
        try{
            system("rosnode kill joint_state_publisher");
            system("rosnode kill robot_state_publisher");
            system("rosnode kill gazebo");
            system("rosnode kill usb_cam");
            system("killall -9 gzserver");
            ros_launch_manager.stop(hardware_launcher, SIGINT);
        }
        catch (std::exception const &exception) {
            ROS_WARN("%s", exception.what());
        }
    }
    else{
        ROS_INFO("no command received");
    }
    return true;
}

bool cameraExposureUpdate(ros_srv::CameraExposure::Request &req, ros_srv::CameraExposure::Response &res){
    ROS_INFO("Got it! Camera Exposure Updated: %ld", req.command);
    res.success = true;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hardware_signal");
    ros::CallbackQueue callback_queue;
    ros::NodeHandlePtr n_;
    n_.reset(new ros::NodeHandle("~"));
    n_->setCallbackQueue(&callback_queue);
    callback_queue.callAvailable(ros::WallDuration());

    ros::ServiceServer velodyneSwitchService = n_->advertiseService("velodyneSwitch", velodyneSwitch);
    ros::ServiceServer cameraExposureUpdateService = n_->advertiseService("cameraExposureUpdate", cameraExposureUpdate);
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin(&callback_queue);

    return 0;
}


