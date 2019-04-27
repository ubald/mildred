#include "StandingState.h"

#include <math.h>

namespace Mildred {
    StandingState::StandingState(MildredControl *control) : ControlState(MildredState::Standing, "standing", control) {}

    bool StandingState::onEnter(const Event &event) {
        for (const auto &leg:control_->body->legs) {
            auto fk = leg->doFK();
            if (!fk.first) {
                ROS_ERROR_STREAM("Failed to compute FK for leg " << leg->name << " while standing.");
                return false;
            }

            ROS_INFO("FK POS: %f %f %f", fk.second.x(), fk.second.y(), fk.second.z());

            leg->targetPosition = fk.second;
            trajectories.emplace_back(leg->targetPosition, leg->frame * KDL::Vector(0.15f, 0.00f, 0.00f));
        }

        moving = true;

        return true;
    }

    bool StandingState::onExit(const Ragdoll &event) {
        // Ragdoll is a safety event, we don't cancel transition when received
        trajectories.clear();
        return true;
    }

    bool StandingState::onExit(const Event &event) {
        if (moving) {
            return false;
        }

        trajectories.clear();
        return true;
    }

    void StandingState::tick(double now, double delta) {
        if (startTime == 0.00f) {
            startTime = now;
        }

        // TODO: Rewrite, this is testing bullshit

        if (moving) {
            // Step 1 - Regroup Limbs
            // TODO: If all within eps, skip the timer and go to standing
            double          t     = std::fmin((now - startTime) / 2.00f, 1.00f);
            uint8_t         index = 0;
            for (auto const &leg:control_->body->legs) {
                leg->targetPosition.x(trajectories[index].first.x() + (trajectories[index].second.x() - trajectories[index].first.x()) * t);
                leg->targetPosition.y(trajectories[index].first.y() + (trajectories[index].second.y() - trajectories[index].first.y()) * t);
                leg->targetPosition.z(trajectories[index].first.z() + (trajectories[index].second.z() - trajectories[index].first.z()) * t);
                leg->doIK();
                index++;
            }

            if (t == 1.00f) {
                startTime  = now;
                moving = false;
                trajectories.clear();
                for (const auto &leg:control_->body->legs) {
                    trajectories.emplace_back(leg->targetPosition, leg->frame * KDL::Vector(0.15f, 0.00f, -0.14f));
                }
            }
        } else {
            // Step 2 - Stand
            double          t     = std::fmin((now - startTime) / 0.50f, 1.00f);
            uint8_t         index = 0;
            for (auto const &leg:control_->body->legs) {
                leg->targetPosition.x(trajectories[index].first.x() + (trajectories[index].second.x() - trajectories[index].first.x()) * t);
                leg->targetPosition.y(trajectories[index].first.y() + (trajectories[index].second.y() - trajectories[index].first.y()) * t);
                leg->targetPosition.z(trajectories[index].first.z() + (trajectories[index].second.z() - trajectories[index].first.z()) * t);
                leg->doIK();
                index++;
            }
        }
    }
}