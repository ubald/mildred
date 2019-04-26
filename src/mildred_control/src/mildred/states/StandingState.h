#pragma once

#include <kdl/frames.hpp>
#include <tf2/LinearMath/Vector3.h>

#include <mildred_control/fsm/State.h>
#include <mildred_control/MildredControl.h>

#include "events.h"

namespace Mildred {
    class StandingState : public State {
        public:
            StandingState(MildredControl * control);
            ~StandingState() = default;

            bool onEnter(const Event &event) override;
            bool onExit(const Event &event) override;
            void tick(double now, double delta) override;

        protected:
            MildredControl *control{nullptr};

            double startTime = 0.00f;
            bool regrouping = true;
            std::vector<std::pair<KDL::Vector, KDL::Vector>> trajectories;
    };
}