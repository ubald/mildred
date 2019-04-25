#include <mildred_core/ActuatorsStateMessage.h>
#include "IdleState.h"

namespace Mildred {
    IdleState::IdleState(MildredControl * control) :
        State(),
        control(control){}

    bool IdleState::onEnter(const Mildred::Event &event) {
        mildred_core::ActuatorsStateMessage message;
        message.state = 0;
        control->actuatorsStatePublisher.publish(message);
        control->publishJointPositions = false;
        return true;
    }

    bool IdleState::onExit(const Mildred::Event &event) {
        for (auto const &leg:control->body->legs) {
            for (auto &joint:leg->joints) {
                joint.targetPosition = joint.currentPosition;
            }
        }

        mildred_core::ActuatorsStateMessage message;
        message.state = 1;
        control->actuatorsStatePublisher.publish(message);
        control->publishJointPositions = true;
        return true;
    }
}