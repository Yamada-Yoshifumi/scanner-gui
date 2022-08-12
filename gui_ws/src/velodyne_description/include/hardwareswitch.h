#ifndef HARDWARESWITCH_H
#define HARDWARESWITCH_H
#include <ros/ros.h>
#include "ros_srv/VelodyneSwitch.h"
#include "roslaunchmanager.h"

class HardwareSwitch
{
private:
    ros::NodeHandlePtr n_;
    ros::ServiceServer velodyneSwitchService;


public:
    HardwareSwitch();
    static bool velodyneSwitch(ros_srv::VelodyneSwitch::Request &req, ros_srv::VelodyneSwitch::Response &res)
    {
        ROSLaunchManager ros_launch_manager;
        pid_t hardware_interface;
        if (req.command == 1){
            try {
                hardware_interface = ros_launch_manager.start(
                    "velodyne_description", "example.launch");
            }
            catch (std::exception const &exception) {
                ROS_WARN("%s", exception.what());
                ros_launch_manager.stop(hardware_interface, SIGINT);
                res.success = false;
                return res.success;
            }
            res.success = true;
            return res.success;
        }
        else if (req.command == 0){
            try {
                ros_launch_manager.stop(hardware_interface, SIGINT);
            }
            catch (std::exception const &exception) {
                ROS_WARN("%s", exception.what());
                res.success = false;
                return res.success;
            }
            res.success = true;
            return res.success;
        }
        else{
            res.success = false;
            return res.success;
        }
    }
};

#endif // HARDWARESWITCH_H
