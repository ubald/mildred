#include "IdleState.h"

namespace Mildred {
    IdleState::IdleState(Body * body) :
        State(),
        body(body){

    }

    bool IdleState::onEnter(const Mildred::Event &event) {
        body->turnActuatorsOff();
    }

}