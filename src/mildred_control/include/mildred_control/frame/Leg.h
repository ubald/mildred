#pragma once

#include <memory>

#include <Eigen/Core>

#include <ros/ros.h>

#include <urdf/model.h>
#include <urdf_model/link.h>
#include <urdf_model/joint.h>

//#include "mildred_control/ik/UChainIkSolverPosNRJL.h"

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include <control_toolbox/pid.h>

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
        bool setup(std::shared_ptr<urdf::Model> model, std::unique_ptr<KDL::Chain> chain, std::string root, std::string tip);

        /**
         * Set the current gait the leg will use when walking.
         *
         * @param gait
         */
        void setGait(std::shared_ptr<Mildred::Gait> gait);


        KDL::Vector               targetPosition;
        KDL::Frame                frame;
        static const unsigned int JOINT_COUNT = 3;
        Mildred::Joint            joint[JOINT_COUNT];
        KDL::Vector doGait();
        bool doIK(KDL::Vector target);

    protected:
        ros::NodeHandle                n;
        std::shared_ptr<Mildred::Gait> currentGait{nullptr};
        //double               lastPIDTime;
        //control_toolbox::Pid pidX;
        //control_toolbox::Pid pidY;
        //control_toolbox::Pid pidZ;

    private:
        bool                                             init_run;
        unsigned int                                     jointCount;
        KDL::JntArray                                    q_init;
        KDL::JntArray                                    q_out;
        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver{nullptr};
        std::unique_ptr<KDL::ChainIkSolverVel_wdls>      ik_solver_vel{nullptr};
        std::unique_ptr<KDL::ChainIkSolverPos_NR_JL>     ik_solver_pos{nullptr};
    };

}
