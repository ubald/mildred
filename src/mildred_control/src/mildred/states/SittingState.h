#pragma once

#include "mildred_control/fsm/State.h"
#include "mildred_control/MildredControl.h"

namespace Mildred {
    class SittingState: public State {
        public:
            SittingState(MildredControl * control);
            ~SittingState() = default;

            std::string name() const override { return "sitting"; }

        protected:
            MildredControl *control{nullptr};
    };
}
