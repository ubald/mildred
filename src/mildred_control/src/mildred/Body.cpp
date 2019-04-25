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

        return true;
    }

    void Body::setJointState(const sensor_msgs::JointState::ConstPtr& jointState) {
        for (const auto &leg:legs) {
            leg->setJointState(jointState);
        }
    }

    void Body::tick(double now, double delta) {
        for (const auto &leg:legs) {
            leg->tick(now, delta);
        }
    }
}
