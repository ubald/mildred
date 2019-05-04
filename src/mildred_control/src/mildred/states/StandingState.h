#pragma once

#include <tf2/LinearMath/Vector3.h>

#include <mildred_control/mildred/states/ControlState.h>

#include "events.h"

namespace Mildred {
    class StandingState : public ControlState {
      public:
        StandingState(MildredControl *control);
        ~StandingState() = default;

        bool onEnter(const Event &event) override;
        bool onExit(const Ragdoll &event);
        bool onExit(const Event &event) override;
        void tick(double now, double delta) override;

      protected:
        bool moving = true;
        std::vector<tf2::Vector3> targetPositions;
    };
}