#include <memory>

#include <ros/ros.h>

#include <mildred_teleop/MildredTeleopJoyPS4.h>

using namespace Mildred;

int main(int argc, char **argv) {
    ros::init(argc, argv, "mildred_teleop");
    ros::start();

    auto mildredTeleop = std::make_unique<MildredTeleopJoyPS4>();

    ros::spin();
    ros::shutdown();

    return 0;
}