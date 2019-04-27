#include <utility>

#include <mildred_control/MildredControl.h>
#include <mildred_control/mildred/states/ControlState.h>
#include <mildred_control/mildred/states/ControlMachine.h>

namespace Mildred {
    ControlState::ControlState(MildredState id, std::string name, Mildred::MildredControl *control):
    State(std::move(name)), id_(id), control_(control) {}

    void ControlState::handleControl(const mildred_core::MildredControlMessage::ConstPtr &controlMessage) {}
}