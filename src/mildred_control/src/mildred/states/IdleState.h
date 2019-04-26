#pragma once

#include "mildred_control/fsm/State.h"
#include "mildred_control/MildredControl.h"

#include "events.h"

namespace Mildred {
    class IdleState : public State {
        public:
            IdleState(MildredControl * control);
            ~IdleState() = default;
            bool onEnter(const Event &event) override;
            bool onExit(const Event &event) override;

        protected:
            MildredControl *control{nullptr};
    };
}