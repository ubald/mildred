#include "WalkingState.h"

namespace Mildred {
    WalkingState::WalkingState(MildredControl *control) : ControlState(MildredState::Walking, "walking", control) {}

    bool WalkingState::onEnter(const Event &event) {
        setGait(CONTINUOUS, RIPPLE);
        return true;
    }

    void WalkingState::tick(double now, double delta) {
        gait->prepare(targetSpeed, targetDirection);

        /**
         * Compute Gait and IK on each leg
         */
        for (const auto &leg:control_->body->legs) {
            KDL::Vector gaitStep       = leg->doGait();
            KDL::Vector positionInBody = control_->body->frame * gaitStep;
            ROS_DEBUG_STREAM("Leg " << leg->name << ":");
            ROS_DEBUG_STREAM(" - Gait: " << gaitStep.x() << ", " << gaitStep.y() << ", " << gaitStep.z());
            ROS_DEBUG_STREAM(" - Body: " << positionInBody.x() << ", " << positionInBody.y() << ", " << positionInBody.z());
            leg->doIK(positionInBody);
        }
    }

    void WalkingState::handleControl(const mildred_core::MildredControlMessage::ConstPtr &controlMessage) {
        auto velocity = KDL::Vector2(controlMessage->velocity.x, controlMessage->velocity.y);

        // A check is first made to see if an axis' value is 0.00 because the Norm()
        // method doesn't check that and returns NaN.
        if (velocity.x() == 0.00) {
            targetSpeed = fabs(velocity.y());
        } else if (velocity.y() == 0.00) {
            targetSpeed = fabs(velocity.x());
        } else {
            targetSpeed = fabs(velocity.Norm());
        }

        // fmin to 1.0 because the PS3 remote doesn't go in a circle
        // but rather in a square with rounded corners at about x=0.9, y=0.9
        targetSpeed     = fmin(targetSpeed, 1.00);
        targetDirection = atan2(velocity.y(), velocity.x());

        ROS_DEBUG("Speed: %f Direction: %f", targetSpeed, targetDirection);

        //body->frame    = KDL::Frame(
        //    KDL::Rotation::RPY(
        //        controlMessage->rotation.y * (M_PI / 8), //TODO: Dynamically adjust maximum rotation
        //        controlMessage->rotation.x * (M_PI / 8), //TODO: Dynamically adjust maximum rotation
        //        controlMessage->rotation.z * (M_PI / 8)  //TODO: Dynamically adjust maximum rotation
        //    ),
        //    KDL::Vector(
        //        controlMessage->position.x / 10.00, //TODO: Dynamically adjust maximum position
        //        controlMessage->position.y / 10.00, //TODO: Dynamically adjust maximum position
        //        controlMessage->position.z / 10.00  //TODO: Dynamically adjust maximum position
        //    )
        //);
    }

    void WalkingState::setGait(Mildred::EGaitShape shape, Mildred::EGaitSequence sequence) {
        switch (shape) {
            case CONTINUOUS:
            default:
                gait = std::make_shared<Mildred::ContinuousGait>(sequence);
                break;
        }

        /**
         * Setup the gait
         * NOTE Is this really needed or just the constructor would be enough?
         */
        gait->setup();

        /**
         * Assign the gait to each leg
         * NOTE Is this really needed, do the legs need to know the gait?
         */
        for (const auto &leg:control_->body->legs) {
            leg->setGait(gait);
        }
    }

}