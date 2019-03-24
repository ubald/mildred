#include "mildred_control/frame/Leg.h"

namespace Mildred {
    Leg::Leg(unsigned int legIndex) {
        gaitConfig.index = legIndex;
    }

    bool Leg::setup(urdf::Model model, KDL::Chain chain, std::string root, std::string tip) {
        //Setup pid
        //lastPIDTime = ros::Time::now().sec + (ros::Time::now().nsec * 1e-9);
        //pidX.initPid(0.05, 0.00, 0.00, 0, 0);
        //pidY.initPid(0.05, 0.00, 0.00, 0, 0);
        //pidZ.initPid(0.05, 0.00, 0.00, 0, 0);

        //Name our leg, for easier identification in debug info
        name = chain.segments[0].getName();

        //Compose root frame and leg/hip frame
        //This is highly dependent on our specific URDF model design
        frame = chain.segments[0].getFrameToTip() * chain.segments[1].getFrameToTip();

        //Gait configuration
        double r, p, y;
        frame.M.GetRPY(r, p, y);
        gaitConfig.alpha = y;

        //Initialize the total number of joints
        num_joints = 0;

        /**
         * Get the joints limits
         */

        auto link = model.getLink(tip);
        std::shared_ptr<const urdf::Joint> link_joint;

        //Count joints and check that chain isn't broken
        //NOTE: Taken from other source, could be improved a lot, like root name could be found in the chain
        while (link && link->name != root) {
            link_joint = model.getJoint(link->parent_joint->name);
            if (!link_joint) {
                ROS_ERROR("Leg: Failed to find joints");
                return false;
            }
            if (link_joint->type != urdf::Joint::UNKNOWN && link_joint->type != urdf::Joint::FIXED) {
                ROS_INFO("Adding joint: %s", link_joint->name.c_str());
                num_joints++;
            }
            link = model.getLink(link->getParent()->name);
        }

        joint_min.resize(num_joints);
        joint_max.resize(num_joints);

        //Re-traverse the chain and save limits
        link = model.getLink(tip);
        unsigned int i = 0;
        while (link && i < num_joints) {
            link_joint = model.getJoint(link->parent_joint->name);
            if (link_joint->type != urdf::Joint::UNKNOWN && link_joint->type != urdf::Joint::FIXED) {
                ROS_INFO("Getting bounds for joint: %s", link_joint->name.c_str());

                //Save name to our joint object
                joint[i].name = link_joint->name;
                //TODO: Save actual joint pointer, create new UWalker::Joint here...

                //Get limits
                //TODO: Modify our solver to support a UWalker::Joint array instead and save these limits to our joint
                double lower, upper;
                //int hasLimits;
                if (link_joint->type != urdf::Joint::CONTINUOUS) {
                    lower = link_joint->limits->lower;
                    upper = link_joint->limits->upper;
                    //hasLimits = 1;
                } else {
                    lower = -M_PI;
                    upper = M_PI;
                    //hasLimits = 0;
                }
                int index = num_joints - i - 1;
                joint_min.data[index] = lower;
                joint_max.data[index] = upper;
                i++;
            }
            link = model.getLink(link->getParent()->name);
        }

        //Build Solvers
        fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);
        ik_solver_vel = std::make_unique<KDL::ChainIkSolverVel_wdls>(chain);

        //Only the end effector position is needed - Task Space
        //NOTE: I don't know the fuck this does but without it it can't solve...
        Eigen::MatrixXd matrix_Mx = Eigen::MatrixXd::Identity(6, 6);
        matrix_Mx(3, 3) = 0;
        matrix_Mx(4, 4) = 0;
        matrix_Mx(5, 5) = 0;
        ik_solver_vel->setWeightTS(matrix_Mx);
        ik_solver_pos = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(chain, joint_min, joint_max, *fk_solver, *ik_solver_vel, 1000, 0.001);

        //Resize our q_init array
        //NOTE: This used to be in doIK but I fear it was affecting performance
        q_init.resize(num_joints);

        //Assign initialization values, if IK fails on startup look here...
        /*init_run = true;
         for ( unsigned int j = 0; j < num_joints; j++)
         {
         q_init(j) = M_PI / 2;
         }*/

        //Default joint init values for IK
        joint[0].targetPosition = 0.00;
        joint[1].targetPosition = -M_PI_2;
        joint[2].targetPosition = 3 * M_PI_4;

        return true;
    }

    void Leg::setGait(std::shared_ptr<Mildred::Gait> gait) {
        //TODO: Wait for the right moment to switch gait
        //TODO: Implement state machine with transitions
        currentGait = gait;
    }

    KDL::Vector Leg::doGait() {
        //Step in the gait with our gait configuration
        KDL::Vector gaitTargetPosition = currentGait->walk(gaitConfig);

        //Compose position in leg frame
        KDL::Vector positionInLegFrame = frame * gaitTargetPosition;

        //TODO Other leg-position-specific shit here, ground contact, behavior whatever

        //Return final target position
        return positionInLegFrame;
    }

    bool Leg::doIK(KDL::Vector target) {
        //Save the target position
        //double        now = ros::Time::now().sec + (ros::Time::now().nsec * 1e-9);
        //ros::Duration d   = ros::Duration(now - lastPIDTime);
        //targetPosition = KDL::Vector(
        //    pidX.updatePid(targetPosition.x() - target.x(), d),
        //    pidY.updatePid(targetPosition.y() - target.y(), d),
        //    pidZ.updatePid(targetPosition.z() - target.z(), d)
        //);
        ////ROS_ERROR_STREAM( "x:" << targetPosition.x() << " y:" << targetPosition.y() << " z:" << targetPosition.z() );
        //lastPIDTime    = now;

        //Frame the target position
        KDL::Frame p_in = KDL::Frame(target);

        //Get the current joint positions (our array is base->tip, IK works with tip->base)
        for (unsigned int i = 0; i < JOINT_COUNT; i++) {
            std::cout << joint[i].currentPosition << std::endl;
            q_init((JOINT_COUNT - 1) - i) = joint[i].currentPosition;
        }

        int ik_valid = ik_solver_pos->CartToJnt(q_init, p_in, q_out);
        // Only when a solution is found it will be sent
        if (ik_valid >= 0) {
            init_run = false;
            for (unsigned int i = 0; i < num_joints; i++) {
                joint[i].targetPosition = q_out(i);
            }
        } else {
            ROS_WARN("Leg::doIK() IK Solution not found for : %s, error: %d", name.c_str(), ik_valid);
            ROS_DEBUG("Leg::doIK() IK POS: %f %f %f", target.x(), target.y(), target.z());
            init_run = true;
        }

        return (ik_valid >= 0);
    }
}
