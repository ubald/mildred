#pragma once

#include "mildred_control/mildred/states/ControlState.h"
#include "mildred_control/gait/gaits.h"
#include "mildred_control/gait/Gait.h"

namespace Mildred {
    class WalkingState : public ControlState {
      public:
        WalkingState(MildredControl *control);
        ~WalkingState() = default;

        bool onEnter(const Event &event) override;
        bool onExit(const Event &event) override;
        void tick(double now, double delta) override;
        void handleControl(const mildred_core::MildredControlMessage::ConstPtr &controlMessage) override;

      protected:
        void setGait(GaitShape shape, GaitSequence sequence);
        std::shared_ptr<Gait> gait{nullptr};

        double targetSpeed;
        double targetDirection;
        bool preparing;
        std::vector<tf2::Vector3> targetPositions;
    };
}
