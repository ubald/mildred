#include "mildred_control/mildred/Body.h"

namespace Mildred {
    Body::Body() {}

    bool Body::setup(std::shared_ptr<urdf::Model> model, uint8_t legCount, std::string legTipPrefix) {
        auto tree = std::make_unique<KDL::Tree>();
        if (!kdl_parser::treeFromUrdfModel(*model, *tree)) {
            ROS_ERROR("Failed to initialize tree object");
            return false;
        }

        //Assign chains to legs
        for (unsigned int i = 0; i < legCount; ++i) {
            std::string tipName = legTipPrefix + "_" + std::to_string(i);

            auto chain = std::make_unique<KDL::Chain>();
            ROS_INFO("Getting chain from \"base_link\" to \"%s\"", tipName.c_str());
            if (!tree->getChain("base_link", tipName, *chain)) {
                ROS_ERROR("Failed get chain from \"base_link\" to \"%s\"", tipName.c_str());
                return false;
            }

            auto leg = std::make_shared<Mildred::Leg>(i);
            if (!leg->setup(model, std::move(chain), tipName)) {
                ROS_ERROR("Failed to setup Leg");
                return false;
            }

            legs.emplace_back(leg);
        }

        /**
         * Set the startup, default gait
         * NOTE: This is probably going to change as a startup routine is implemented
         */
        setGait(CONTINUOUS, RIPPLE);

        return true;
    }

    void Body::setJointState(const sensor_msgs::JointState::ConstPtr& jointState) {
        for (const auto &leg:legs) {
            leg->setJointState(jointState);
        }
    }

    void Body::setGait(Mildred::EGaitShape shape, Mildred::EGaitSequence sequence) {
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
        for (const auto &leg:legs) {
            leg->setGait(gait);
        }
    }

    void Body::tick() {
        // A check is first made to see if an axis' value is 0.00 because the Norm()
        // method doesn't check that and returns NaN.
        if (velocity.x() == 0.00) {
            speed = fabs(velocity.y());
        } else if (velocity.y() == 0.00) {
            speed = fabs(velocity.x());
        } else {
            speed = fabs(velocity.Norm());
        }

        // fmin to 1.0 because the PS3 remote doesn't go in a circle
        // but rather in a square with rounded corners at about x=0.9, y=0.9
        speed = fmin(speed, 1.00);
        direction = atan2(velocity.y(), velocity.x());

        ROS_DEBUG("Speed: %f Direction: %f", speed, direction);

        gait->prepare(speed, direction);

        /**
         * Compute Gait and IK on each leg
         */
        for (const auto &leg:legs) {
            KDL::Vector gaitStep = leg->doGait();
            KDL::Vector positionInBody = frame * gaitStep;
            ROS_DEBUG_STREAM("Leg " << leg->name << ":");
            ROS_DEBUG_STREAM(" - Gait: " << gaitStep.x() << ", " << gaitStep.y() << ", " << gaitStep.z());
            ROS_DEBUG_STREAM(" - Body: " << positionInBody.x() << ", " << positionInBody.y() << ", " << positionInBody.z());
            leg->doIK(positionInBody);
        }
    }
}
