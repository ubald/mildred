#pragma once

#include "mildred_control/mildred/states/ControlState.h"

namespace Mildred {
    class WalkingState : public ControlState {
      public:
        WalkingState(MildredControl *control);
        ~WalkingState() = default;

        bool onEnter(const Event &event) override;
        void tick(double now, double delta) override;
        void handleControl(const mildred_core::MildredControlMessage::ConstPtr &controlMessage) override;

      protected:
        void setGait(Mildred::EGaitShape shape, Mildred::EGaitSequence sequence);
        std::shared_ptr<Mildred::Gait> gait{nullptr};

        double targetSpeed;
        double targetDirection;
    };
}
