#pragma once

#include <mildred_control/mildred/states/ControlState.h>

#include "events.h"

namespace Mildred {
    class IdleState : public ControlState {
        public:
            IdleState(MildredControl * control);
            ~IdleState() = default;
            bool onEnter(const Event &event) override;
            bool onExit(const Event &event) override;
    };
}