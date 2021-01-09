#include <memory>
#include <ros/ros.h>
#include "mildred_control/MildredControl.h"

using namespace Mildred;

int main(int argc, char **argv) {
    ros::init(argc, argv, "mildred_control");
    ros::start();
    ros::Rate rate(100);

    auto mildredControl = std::make_unique<MildredControl>();
    if (!mildredControl->init()) {
        ROS_ERROR("Error during initialization");
        return 1;
    }

    while (ros::ok()) {
        ros::spinOnce();
        mildredControl->loop();
        rate.sleep();
    }

    ros::shutdown();

    return 0;
}