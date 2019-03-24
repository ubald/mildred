#pragma once

#include <memory>
#include <vector>

#include <ros/ros.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>

#include "mildred_control/gait/gaits.h"

#include "Leg.h"

namespace Mildred {
    class Body {
    public:
        Body();
        ~Body() = default;

        static const unsigned int LEG_COUNT = 6;
        std::vector<std::shared_ptr<Mildred::Leg>> legs;

        bool setup(urdf::Model model, std::string root, std::string leg_tip_prefix);
        void setGait(Mildred::EGaitShape shape, Mildred::EGaitSequence sequence);

        void tick();

        KDL::Rotation rotation;
        KDL::Frame    frame;
        KDL::Vector2  velocity;

    protected:
        ros::NodeHandle n;
        KDL::Tree       tree;

        std::shared_ptr<Mildred::Gait> gait;

        double speed;
        double direction;
    };
}
