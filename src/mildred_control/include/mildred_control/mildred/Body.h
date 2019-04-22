#pragma once

#include <memory>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>

#include "mildred_control/gait/gaits.h"

#include "Leg.h"

namespace Mildred {

    class Body {
        public:
            Body();
            ~Body() = default;

            bool setup(std::shared_ptr<urdf::Model> model, uint8_t legCount, std::string legTipPrefix);
            void setJointState(const sensor_msgs::JointState::ConstPtr &jointState);
            void setGait(Mildred::EGaitShape shape, Mildred::EGaitSequence sequence);

            void turnActuatorsOff();

            void tick();

            std::vector<std::shared_ptr<Mildred::Leg>> legs;

            KDL::Rotation rotation;
            KDL::Frame    frame;
            KDL::Vector2  velocity;

        protected:
            std::shared_ptr<Mildred::Gait> gait{nullptr};

            double speed;
            double direction;
    };
}
