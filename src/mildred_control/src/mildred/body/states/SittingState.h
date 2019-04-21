#pragma once

#include <string>

#include "mildred_control/fsm/State.h"

namespace Mildred {
    class SittingState: public State {
        public:
            std::string name() const override { return "sitting"; }
    };
}
