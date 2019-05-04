#pragma once

#include <tf2/LinearMath/Vector3.h>

#include <mildred_control/mildred/states/ControlState.h>

#include "events.h"

namespace Mildred {
    class SittingState : public ControlState {
      public:
        SittingState(MildredControl *control);
        ~SittingState() = default;

        bool onEnter(const Event &event) override;
        bool onExit(const Ragdoll &event);
        bool onExit(const Event &event) override;
        void tick(double now, double delta) override;

      protected:
        double startTime = 0.00f;
        bool   moving    = true;

        std::vector<std::pair<tf2::Vector3, tf2::Vector3>> trajectories;
    };
}
