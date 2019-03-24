#include "mildred_control/frame/Joint.h"

namespace Mildred {
    Joint::Joint() {
        //Default positions
        currentPosition = targetPosition = 0.00;

        //TODO: Setup servo

        //Joint state subscriber
        // TODO: Fix the hardcoded namespace
        jointStatesSubscriber = n.subscribe("/mildred/joint_states", 1, &Joint::jointsStatesCallback, this);
    }

    /**
     * Callback getting the joint's current position from the jointStates messages
     */
    void Joint::jointsStatesCallback(const JointStateConstPtr &jointStatesMessage) {
        for (unsigned int i = 0; i < jointStatesMessage->name.size(); i++) {
            if (jointStatesMessage->name[i] == name) {
                currentPosition = jointStatesMessage->position[i];
                break;
            }
        }
    }
}
