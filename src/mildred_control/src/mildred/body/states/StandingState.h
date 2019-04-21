#pragma once

#include "mildred_control/fsm/State.h"
#include "events.h"

namespace Mildred {
    class StandingState : public State {
        public:
            StandingState();
            ~StandingState() = default;

            std::string name() const override { return "standing"; }

            bool onEnter(const Stand &event);
            bool onEnter(const Event &event) override;
    };
}