#pragma once

#include <vector>
#include <algorithm>

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>
#include <sensor_msgs/JoyFeedbackArray.h>

#include "mildred_teleop/MildredTeleop.h"

namespace Mildred {

    class MildredTeleopJoy : public MildredTeleop {
    protected:
        MildredTeleopJoy(std::size_t buttonCount, std::size_t axesCount);
        ~MildredTeleopJoy() = default;
        virtual void joyCallback(sensor_msgs::Joy joyMsg);
        std::vector<bool> lastButtonValue;
        std::vector<float> lastAxisValue;
        std::vector<bool> axisChanged;
    private:
        ros::Subscriber joySubscriber;
        ros::Publisher  joyFeedbackPublisher;
    };

}