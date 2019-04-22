#pragma once

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

namespace Mildred {
    class Joint {
    public:
        Joint();
        ~Joint() = default;

        void setJointState(const sensor_msgs::JointState::ConstPtr &jointState);
        std::string name;

        double targetPosition  = 0.f;
        double currentPosition = 0.f;
        double currentVelocity = 0.f;
        double currentEffort   = 0.f;
    };
}