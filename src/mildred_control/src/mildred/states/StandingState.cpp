#include "StandingState.h"

namespace Mildred {
    StandingState::StandingState(MildredControl * control) :
        State(),
        control(control){}

    bool StandingState::onEnter(const Stand &event) {
        std::cout << "stand event enter" << std::endl;
        return true;
    }

    bool StandingState::onEnter(const Event &event) {
        std::cout << "generic enter" << std::endl;
        return true;
    }

    void StandingState::tick(double now, double delta) {
        for (const auto &leg:control->body->legs) {
            std::cout << "leg" << std::endl;
            leg->doFK();
        }
    }
}