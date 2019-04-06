#pragma once

#include <ros/ros.h>

#include <mildred_core/RemoteControlMessage.h>

namespace Mildred {
    class MildredTeleop {
    protected:
        MildredTeleop();
        ~MildredTeleop() = default;
        ros::NodeHandle nodeHandle;
    protected:
        ros::Publisher                     controlPublisher;
        mildred_core::RemoteControlMessage controlMessage;
    };

}