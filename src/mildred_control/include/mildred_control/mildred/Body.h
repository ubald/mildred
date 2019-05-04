#pragma once

#include <memory>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <tf2/LinearMath/Transform.h>

#include "mildred_control/gait/gaits.h"

#include "Leg.h"

namespace Mildred {

    class Body {
        public:
            Body();
            ~Body() = default;

            bool setup(std::shared_ptr<urdf::Model> model, uint8_t legCount, std::string legTipPrefix);
            void setJointState(const sensor_msgs::JointState::ConstPtr &jointState);

            void tick(double now, double delta);

            std::vector<std::shared_ptr<Mildred::Leg>> legs;

            tf2::Transform frame;

        protected:
    };
}
