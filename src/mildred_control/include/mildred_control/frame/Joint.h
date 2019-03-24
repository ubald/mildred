#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

typedef boost::shared_ptr<sensor_msgs::JointState const> JointStateConstPtr;

namespace Mildred {
    class Joint {
    public:
        Joint();
        ~Joint() = default;


        std::string name;
        double      targetPosition;
        double      currentPosition;

    protected:
        ros::NodeHandle n{};

        ros::Subscriber jointStatesSubscriber;
        void jointsStatesCallback(const JointStateConstPtr &jointStatesMessage);
    };
}