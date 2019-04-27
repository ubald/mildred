#pragma once

#include "mildred_control/mildred/states/ControlState.h"

namespace Mildred {
    class StartingState : public ControlState {
        public:
            StartingState(MildredControl * control);
            ~StartingState() = default;
    };
}