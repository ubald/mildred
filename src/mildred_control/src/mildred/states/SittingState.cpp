#include "SittingState.h"

namespace Mildred {
    SittingState::SittingState(MildredControl *control) : ControlState(MildredState::Sitting, "sitting", control) {}

    bool SittingState::onEnter(const Event &event) {
        for (const auto &leg:control_->body->legs) {
            leg->targetPosition = leg->currentPosition;
            targetPositions.emplace_back(leg->frame * tf2::Vector3(0.15f, 0.00f, 0.00f));
        }

        moving = true;

        return true;
    }

    bool SittingState::onExit(const Ragdoll &event) {
        // Ragdoll is a safety event, we don't cancel transition when received
        targetPositions.clear();
        return true;
    }

    bool SittingState::onExit(const Event &event) {
        if (moving) {
            return false;
        }

        targetPositions.clear();
        return true;
    }

    void SittingState::tick(double now, double delta) {
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