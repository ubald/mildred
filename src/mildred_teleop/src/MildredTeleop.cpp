#include "mildred_teleop/MildredTeleop.h"

namespace Mildred {
    MildredTeleop::MildredTeleop() :
        nodeHandle() {
        stateSubscriber  = nodeHandle.subscribe("mildred_state", 1, &MildredTeleop::stateMessageCallback, this);

        commandPublisher = nodeHandle.advertise<mildred_core::MildredCommandMessage>("command", 1);
        controlPublisher = nodeHandle.advertise<mildred_core::MildredControlMessage>("control", 1);
    }

    void MildredTeleop::sendCommand(MildredCommand command) {
        mildred_core::MildredCommandMessage message;
        message.command = static_cast<decltype(message.command)>(command);
        commandPublisher.publish(message);
    }

    void MildredTeleop::stateMessageCallback(const mildred_core::MildredStateMessage::ConstPtr &stateMessage) {
        state = static_cast<MildredState>(stateMessage->state);

        switch (state) {
            case MildredState::Unknown:
                ROS_ERROR("Robot changed to an unknown state");
                break;

            case MildredState::Idle:
                ROS_INFO("Robot now in idle state");
                break;

            case MildredState::Sitting:
                ROS_INFO("Robot now in sitting state");
                break;

            case MildredState::Standing:
                ROS_INFO("Robot now in standing state");
                break;

            case MildredState::Walking:
                ROS_INFO("Robot now in walking state");
                break;
        }
    }
}