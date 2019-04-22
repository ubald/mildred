#pragma once

#include "mildred_control/fsm/State.h"
#include "mildred_control/MildredControl.h"

#include "events.h"

namespace Mildred {
    class StandingState : public State {
        public:
            StandingState(MildredControl * control);
            ~StandingState() = default;

            std::string name() const override { return "standing"; }

            bool onEnter(const Stand &event);
            bool onEnter(const Event &event) override;
            void tick(double now, double delta) override;

        protected:
            MildredControl *control{nullptr};
    };
}