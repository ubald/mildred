#pragma once

#include <ros/ros.h>

#include <mildred_core/mildred.h>
#include <mildred_core/MildredCommandMessage.h>
#include <mildred_core/MildredControlMessage.h>
#include <mildred_core/MildredStateMessage.h>

namespace Mildred {
    class MildredTeleop {
      protected:
        MildredTeleop();
        ~MildredTeleop() = default;

        void sendCommand(MildredCommand command);

        ros::NodeHandle nodeHandle;
        ros::Publisher  commandPublisher;
        ros::Publisher  controlPublisher;

        mildred_core::MildredControlMessage controlMessage;

        MildredState state = MildredState::Unknown;

      private:
        ros::Subscriber stateSubscriber;
        void stateMessageCallback(const mildred_core::MildredStateMessage::ConstPtr &stateMessage);
    };

}