#include "SittingState.h"

namespace Mildred {
    SittingState::SittingState(MildredControl *control) : ControlState(MildredState::Sitting, "sitting", control) {}

    bool SittingState::onEnter(const Event &event) {
        startTime = 0.00f;

        for (const auto &leg:control_->body->legs) {
            auto fk = leg->doFK();
            if (!fk.first) {
                ROS_ERROR_STREAM("Failed to compute FK for leg " << leg->name << " while sitting.");
                return false;
            }

            ROS_DEBUG("FK POS: %f %f %f", fk.second.x(), fk.second.y(), fk.second.z());

            leg->targetPosition = fk.second;
            trajectories.emplace_back(leg->targetPosition, leg->frame * KDL::Vector(0.15f, 0.00f, 0.00f));
        }

        moving = true;

        return true;
    }

    bool SittingState::onExit(const Ragdoll &event) {
        // Ragdoll is a safety event, we don't cancel transition when received
        startTime = 0.00f;
        trajectories.clear();
        return true;
    }

    bool SittingState::onExit(const Event &event) {
        if (moving) {
            return false;
        }

        startTime = 0.00f;
        trajectories.clear();
        return true;
    }

    void SittingState::tick(double now, double delta) {
        if (moving) {
            if (startTime == 0.00f) {
                startTime = now;
            }

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
                startTime = now;
                moving    = false;
                trajectories.clear();
            }
        }
    }
}