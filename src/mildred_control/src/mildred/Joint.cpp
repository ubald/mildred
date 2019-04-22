#include "mildred_control/mildred/Joint.h"

namespace Mildred {
    Joint::Joint() {}

    void Joint::setJointState(const sensor_msgs::JointState::ConstPtr& jointState) {
        bool found = false;

        for (unsigned int i = 0; i < jointState->name.size(); ++i) {
            if (jointState->name[i] == name) {
                currentPosition = jointState->position[i];
                currentVelocity = jointState->velocity[i];
                currentEffort = jointState->effort[i];
                found = true;
                break;
            }
        }

        if (!found) {
            ROS_WARN("Could not find joint state for joint \"%s\"", name.c_str());
        }
    }
}
