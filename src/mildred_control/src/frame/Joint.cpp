#include "mildred_control/frame/Joint.h"

namespace Mildred {
    Joint::Joint() {}

    void Joint::setJointState(const sensor_msgs::JointState::ConstPtr& jointState) {
        for (unsigned int i = 0; i < jointState->name.size(); i++) {
            if (jointState->name[i] == name) {
                currentPosition = jointState->position[i];
                currentVelocity = jointState->velocity[i];
                currentEffort = jointState->effort[i];
                break;
            }

            ROS_WARN("Could not find joint state for joint %s", name.c_str());
        }
    }
}
