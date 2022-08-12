#include <ros/ros.h>
#include "hardwareswitch.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "hardware_interface");
    HardwareSwitch* hardwareSwitch = new HardwareSwitch();

    ros::spin();
    return 0;
}
