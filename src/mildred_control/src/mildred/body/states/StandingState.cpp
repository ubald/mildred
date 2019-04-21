#include "StandingState.h"

namespace Mildred {
    StandingState::StandingState() :
        State() {

    }

    bool StandingState::onEnter(const Stand &event) {
        std::cout << "stand event enter" << std::endl;
        return true;
    }

    bool StandingState::onEnter(const Event &event) {
        std::cout << "generic enter" << std::endl;
        return true;
    }
}