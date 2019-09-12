#include "StandingState.h"

#include <math.h>

namespace Mildred {
    StandingState::StandingState(MildredControl *control) : ControlState(MildredState::Standing, "standing", control) {}

    bool StandingState::onEnter(const Event &event) {
        for (const auto &leg:control_->body->legs) {
            leg->targetPosition = leg->currentPosition;
            targetPositions.emplace_back(leg->frame * tf2::Vector3(0.15f, 0.00f, -0.14f));
        }

        moving = true;

        return true;
    }

    bool StandingState::onExit(const Ragdoll &event) {
        // Ragdoll is a safety event, we don't cancel transition when received
        targetPositions.clear();
        return true;
    }

    bool StandingState::onExit(const Event &event) {
        if (moving) {
            return false;
        }

        targetPositions.clear();
        return true;
    }

    void StandingState::tick(double now, double delta) {
        if (moving) {
            double speed    = 0.50f;
            double distance = delta * speed;

            bool            finished = true;
            uint8_t         index    = 0;

            for (auto const &leg:control_->body->legs) {
                tf2::Vector3 move      = targetPositions[index] - leg->currentPosition;
                double       magnitude = fabs(move.length());

                if (magnitude > distance) {
                    finished = false;
                    move *= distance / magnitude;
                    if (index == 0) {
                        ROS_WARN_STREAM(move.length() << " " << distance);
                    }
                }

                leg->targetPosition = leg->currentPosition + move;
                leg->doIK();

                index++;
            }

            if (finished) {
                moving = false;
                targetPositions.clear();
            }
        }
    }
}