#pragma once

#include "mildred_control/fsm/State.h"

#include "events.h"
#include "../../Body.h"

namespace Mildred {
    class IdleState : public State {
        public:
            IdleState(Body * body);
            ~IdleState() = default;
            std::string name() const override { return "idle"; }
            bool onEnter(const Event &event) override;
        protected:
            Body *body{nullptr};
    };
}