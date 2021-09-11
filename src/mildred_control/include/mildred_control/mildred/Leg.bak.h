#pragma once

#include <memory>

#include <Eigen/Core>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <urdf/model.h>
#include <urdf_model/link.h>
#include <urdf_model/joint.h>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

#include "mildred_core/mildred.h"
#include "mildred_control/gait/Gait.h"

#include "Joint.h"

namespace Mildred {
    /**
     * Robot Leg
     */
    class Leg {
    public:

        /**
         * Constructor
         *
         * @param index Index of the leg, used for the gait phases and order.
         */
        explicit Leg(unsigned int index);
        ~Leg() = default;

        /**
         * Name of the leg, used primarily for identification in debug and logs.
         */
        std::string name;

        /**
         * Configuration passed to the gait class to carry to and from the leg
         */
        GaitConfig gaitConfig;

        /**
         * Setup the leg
         *
         * Leg setup is dependent on the urdf model architecture, a change in the robot's
         * architecture mean adjusting this method also.
         *
         * @param model urdf::Model used to retrieve joint limits
         * @param chain KDL::Chain for IK solving
         * @param root  Name of the link to use as the root element
         * @param tip   Name of the tip element
         * @return True on success, or false on failure
         */
        bool setup(std::shared_ptr<urdf::Model> model, std::unique_ptr<KDL::Chain> chain, std::string tip);

        void tick(double now, double delta);

        void setJointState(const sensor_msgs::JointState::ConstPtr &jointState);

        /**
         * Set the current gait the leg will use when walking.
         *
         * @param gait
         */
        void setGait(std::shared_ptr<Mildred::Gait> gait);

        tf2::Transform frame{tf2::Transform::getIdentity()};
        tf2::Vector3   targetPosition;
        tf2::Vector3   currentPosition;
        Mildred::Joint joints[DOF];

        tf2::Vector3 doGait();
        std::pair<bool, tf2::Vector3> doFK();
        bool doIK();
        bool doIK(tf2::Vector3 target);

    protected:
        std::shared_ptr<Mildred::Gait> currentGait{nullptr};

    private:
        bool                                        initRun = true;
        uint8_t                                     jointCount;
        std::unique_ptr<KDL::Chain>                 chain;
        KDL::JntArray                               jointMinimums;
        KDL::JntArray                               jointMaximums;
        KDL::JntArray                               q_init;
        KDL::JntArray                               q_out;
        std::unique_ptr<KDL::ChainFkSolverPos>      fk_solver{nullptr};
        std::unique_ptr<KDL::ChainIkSolverVel_wdls> ik_solver_vel{nullptr};
        std::unique_ptr<KDL::ChainIkSolverPos>      ik_solver_pos{nullptr};
    };

}
