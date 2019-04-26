#pragma once

#include <mildred_control/fsm/State.h>
#include <mildred_control/MildredControl.h>

namespace Mildred {
    class WalkingState: public State {
        public:
            WalkingState(MildredControl * control);
            ~WalkingState() = default;

            bool onEnter(const Event &event) override;
            void tick(double now, double delta) override;
            void handleControl(const mildred_core::MildredControlMessage::ConstPtr &controlMessage) override;

        protected:
            MildredControl *control{nullptr};

            void setGait(Mildred::EGaitShape shape, Mildred::EGaitSequence sequence);
            std::shared_ptr<Mildred::Gait> gait{nullptr};

            double targetSpeed;
            double targetDirection;
    };
}
