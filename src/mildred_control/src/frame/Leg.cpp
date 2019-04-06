#include <utility>

#include "mildred_control/frame/Leg.h"

namespace Mildred {
    Leg::Leg(unsigned int index) :
        name("leg_" + std::to_string(index)) {
        gaitConfig.index = index;
    }

    bool Leg::setup(std::shared_ptr<urdf::Model> model, std::unique_ptr<KDL::Chain> legChain, std::string tip) {
        chain = std::move(legChain);

        ROS_INFO("Setting up leg %s starting at %s", name.c_str(), chain->segments[0].getName().c_str());

        // Compose root frame and leg/hip frame
        // This is highly dependent on our specific URDF model design
        frame = chain->segments[0].getFrameToTip() * chain->segments[1].getFrameToTip();

        // Gait configuration
        double r, p, y;
        frame.M.GetRPY(r, p, y);
        ROS_INFO("    Alpha: %f", y);
        gaitConfig.alpha = y;

        //Initialize the total number of joints
        jointCount = 0;

        // Get the joints limits

        // Count joints and check that chain isn't broken
        auto          modelLink = model->getLink(tip);
        while (modelLink && modelLink->parent_joint) {
            if (modelLink->parent_joint->type != urdf::Joint::UNKNOWN && modelLink->parent_joint->type != urdf::Joint::FIXED) {
                jointCount++;
            }

            modelLink = modelLink->getParent();
        }

        ROS_INFO("Found %d joints", jointCount);
        jointMinimums.resize(jointCount);
        jointMaximums.resize(jointCount);

        // Re-traverse the chain and save limits
        modelLink = model->getLink(tip);
        unsigned int i = 0;
        while (modelLink && modelLink->parent_joint) {
            const auto modelJoint = modelLink->parent_joint;
            if (modelJoint->type != urdf::Joint::UNKNOWN && modelJoint->type != urdf::Joint::FIXED) {
                ROS_INFO("Getting bounds for joint: %s", modelJoint->name.c_str());

                //Save name to our joint object
                joints[i].name = modelJoint->name;

                //Get limits
                double lower, upper;
                if (modelJoint->type != urdf::Joint::CONTINUOUS) {
                    lower = modelJoint->limits->lower;
                    upper = modelJoint->limits->upper;
                } else {
                    lower = -M_PI;
                    upper = M_PI;
                }

                int index = jointCount - i - 1;
                jointMinimums.data[index] = lower;
                jointMaximums.data[index] = upper;
                joints[index].targetPosition = 0.00f;
                i++;
            }

            modelLink = modelLink->getParent();
        }

        //Build Solvers
        fk_solver     = std::make_unique<KDL::ChainFkSolverPos_recursive>(*chain);
        ik_solver_vel = std::make_unique<KDL::ChainIkSolverVel_wdls>(*chain);

        //Only the end effector position is needed - Task Space
        //NOTE: I don't know the fuck this does but without it it can't solve...
        Eigen::MatrixXd matrix_Mx = Eigen::MatrixXd::Identity(6, 6);
        matrix_Mx(3, 3) = 0;
        matrix_Mx(4, 4) = 0;
        matrix_Mx(5, 5) = 0;
        ik_solver_vel->setWeightTS(matrix_Mx);
        //ik_solver_pos = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(*chain, jointMinimums, jointMaximums, *fk_solver, *ik_solver_vel, 1000, 0.01f);
        ik_solver_pos = std::make_unique<KDL::ChainIkSolverPos_NR>(*chain, *fk_solver, *ik_solver_vel, 1000, 0.01f);

        //Resize our q_init array
        q_init.resize(jointCount);
        q_out.resize(jointCount);

        return true;
    }

    void Leg::setJointState(const sensor_msgs::JointState::ConstPtr &jointState) {
        for (auto &joint:joints) {
            joint.setJointState(jointState);
        }
    }

    void Leg::setGait(std::shared_ptr<Mildred::Gait> gait) {
        currentGait = std::move(gait);
    }

    KDL::Vector Leg::doGait() {
        KDL::Vector gaitTargetPosition = currentGait->walk(gaitConfig);
        KDL::Vector positionInLegFrame = frame * gaitTargetPosition;
        //TODO Other leg-position-specific shit here, ground contact, behavior whatever

        //Return final target position
        return positionInLegFrame;
    }

    bool Leg::doIK(KDL::Vector target) {
        targetPosition = target;

        //Save the target position
        //double        now = ros::Time::now().sec + (ros::Time::now().nsec * 1e-9);
        //ros::Duration d   = ros::Duration(now - lastPIDTime);
        //targetPosition = KDL::Vector(
        //    pidX.updatePid(targetPosition.x() - target.x(), d),
        //    pidY.updatePid(targetPosition.y() - target.y(), d),
        //    pidZ.updatePid(targetPosition.z() - target.z(), d)
        //);
        //targetPosition = target;
        ROS_ERROR_STREAM("x:" << target.x() << " y:" << target.y() << " z:" << target.z());
        //lastPIDTime    = now;

        //Get the current joint positions (our array is base->tip, IK works with tip->base)
        for (unsigned int i = 0; i < DOF; i++) {
            q_init((DOF - 1) - i) = joints[i].currentPosition;
        }

        ROS_DEBUG_STREAM_NAMED("kdl", "Input: " << &q_init);

        int ik_valid = ik_solver_pos->CartToJnt(q_init, KDL::Frame(target), q_out);
        // Only when a solution is found it will be sent
        if (ik_valid >= 0) {
            ROS_INFO("VALID!!!");
            init_run            = false;
            for (unsigned int i = 0; i < jointCount; i++) {
                joints[i].targetPosition = q_out(i);
            }
        } else {
            ROS_WARN("Leg::doIK() IK Solution not found for : %s, error: %d", name.c_str(), ik_valid);
            ROS_DEBUG("Leg::doIK() IK POS: %f %f %f", target.x(), target.y(), target.z());
            ROS_DEBUG("Leg::doIK() Q_OUT: %f %f %f", q_out(0), q_out(1), q_out(2));
            init_run = true;

            for (unsigned int i = 0; i < jointCount; i++) {
                joints[i].targetPosition = q_out(i);
            }
        }

        return (ik_valid >= 0);
    }
}
