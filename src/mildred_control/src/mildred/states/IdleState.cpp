#include <mildred_control/MildredControl.h>

#include "IdleState.h"

namespace Mildred {
    IdleState::IdleState(MildredControl *control) : ControlState(MildredState::Idle, "idle", control) {

    }

    bool IdleState::onEnter(const Mildred::Event &event) {
        control_->setActuatorState(false);
        return true;
    }

    bool IdleState::onExit(const Mildred::Event &event) {
        for (auto const &leg:control_->body->legs) {
            for (auto &joint:leg->joints) {
                joint.targetPosition = joint.currentPosition;
            }
        }

        control_->setActuatorState(false);
        return true;
    }
}