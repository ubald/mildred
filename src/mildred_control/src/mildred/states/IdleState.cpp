#include <mildred_core/ActuatorsStateMessage.h>

#include "IdleState.h"

namespace Mildred {
    IdleState::IdleState(MildredControl *control) :
        State(MildredState::Idle, "idle"),
        control(control) {}

    bool IdleState::onEnter(const Mildred::Event &event) {
        control->setActuatorState(false);
        return true;
    }

    bool IdleState::onExit(const Mildred::Event &event) {
        for (auto const &leg:control->body->legs) {
            for (auto &joint:leg->joints) {
                joint.targetPosition = joint.currentPosition;
            }
        }

        control->setActuatorState(false);
        return true;
    }
}