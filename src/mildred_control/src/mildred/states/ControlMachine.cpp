#include <mildred_control/mildred/states/ControlMachine.h>
#include <mildred_control/mildred/states/ControlState.h>

namespace Mildred {
    void ControlMachine::handleControl(const mildred_core::MildredControlMessage::ConstPtr &controlMessage) {
        if (currentState == nullptr) {
            return;
        }

        currentState->handleControl(controlMessage);
    }
}
