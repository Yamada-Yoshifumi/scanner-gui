#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "roslaunchmanager.h"
#include "ros_srv/VelodyneSwitch.h"

bool velodyneSwitch(ros_srv::VelodyneSwitch::Request &req, ros_srv::VelodyneSwitch::Response &res)
{
    ROSLaunchManager ros_launch_manager;
    pid_t hardware_interface;
    res.success = true;
    if (req.command == 1){
        try {
            hardware_interface = ros_launch_manager.start(
                "velodyne_description", "robot_simulation.launch");
        }
        catch (std::exception const &exception) {
            ROS_WARN("%s", exception.what());
            ros_launch_manager.stop(hardware_interface, SIGINT);
            return res.success;
        }
        return res.success;
    }
    else if (req.command == 0){
        try {
            ros_launch_manager.stop(hardware_interface, SIGINT);
        }
        catch (std::exception const &exception) {
            ROS_WARN("%s", exception.what());
            return res.success;
        }
        return res.success;
    }
    else{
        ROS_INFO("no command received");
        return res.success;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hardware_signal");
    ros::CallbackQueue callback_queue;
    ros::NodeHandlePtr n_;
    n_.reset(new ros::NodeHandle("~"));
    n_->setCallbackQueue(&callback_queue);
    callback_queue.callAvailable(ros::WallDuration());

    ros::ServiceServer velodyneSwitchService = n_->advertiseService("velodyneSwitch", velodyneSwitch);
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin(&callback_queue);

    return 0;
}


