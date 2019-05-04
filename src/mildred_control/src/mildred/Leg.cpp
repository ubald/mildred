#include <utility>

#include <mildred_control/mildred/Leg.h>

namespace Mildred {
    Leg::Leg(unsigned int index) :
        name("leg_" + std::to_string(index)) {
        gaitConfig.index = index;
    }

    bool Leg::setup(std::shared_ptr<urdf::Model> model, std::unique_ptr<KDL::Chain> legChain, std::string tip) {
        chain = std::move(legChain);

        ROS_DEBUG("Setting up leg \"%s\" with tip \"%s\"", name.c_str(), tip.c_str());
        for (const auto &segment:chain->segments) {
            ROS_DEBUG_STREAM("  - " << segment.getJoint().getName() << "(" << segment.getJoint().getTypeName() << ")" << " > " << segment.getName());
        }

        // Compose root frame and leg/hip frame
        // This is highly dependent on our specific URDF model design
        auto kdlFrame = chain->segments[0].getFrameToTip() * chain->segments[1].getFrameToTip();

        // Gait configuration
        double r, p, y;
        kdlFrame.M.GetRPY(r, p, y);

        frame.getOrigin().setValue(kdlFrame.p.x(), kdlFrame.p.y(), kdlFrame.p.z());
        frame.getBasis().setRPY(r, p, y);
        //ROS_INFO_STREAM("KDL:" << kdlFrame.p.x() << ", " << kdlFrame.p.y() << ", " << kdlFrame.p.z() << " - " << r << ", " << p << ", " << y);
        //ROS_INFO_STREAM("TF2:" << frame.getOrigin().x() << ", " << frame.getOrigin().y() << ", " << frame.getOrigin().z() << " - " << frame.getRotation().x() << ", " << frame.getRotation().y() << ", " << frame.getRotation().z());
        //return false;

        ROS_DEBUG("  Alpha: %f", y);
        gaitConfig.alpha = y;

        //Initialize the total number of joints
        jointCount = 0;

        // Get the joints limits

        // Count joints and check that chain isn't broken
        auto modelLink = model->getLink(tip);
        while (modelLink && modelLink->parent_joint) {
            if (modelLink->parent_joint->type != urdf::Joint::UNKNOWN && modelLink->parent_joint->type != urdf::Joint::FIXED) {
                jointCount++;
            }

            modelLink = modelLink->getParent();
        }

        ROS_DEBUG("Found %d joints", jointCount);
        jointMinimums.resize(jointCount);
        jointMaximums.resize(jointCount);
        q_init.resize(jointCount);
        q_out.resize(jointCount);

        // Re-traverse the chain and save limits
        modelLink = model->getLink(tip);
        unsigned int i = 0;
        while (modelLink && modelLink->parent_joint) {
            const auto modelJoint = modelLink->parent_joint;
            if (modelJoint->type != urdf::Joint::UNKNOWN && modelJoint->type != urdf::Joint::FIXED) {
                unsigned int index = jointCount - i - 1;
                ROS_DEBUG("Getting bounds for joint %d: %s", index, modelJoint->name.c_str());

                //Save name to our joint object
                joints[index].name           = modelJoint->name;
                joints[index].targetPosition = 0.00f;

                //Get limits
                double lower, upper;
                if (modelJoint->type != urdf::Joint::CONTINUOUS) {
                    lower = modelJoint->limits->lower;
                    upper = modelJoint->limits->upper;
                } else {
                    lower = -M_PI;
                    upper = M_PI;
                }

                ROS_DEBUG_STREAM("  - Lower: " << lower);
                jointMinimums(index) = lower;

                ROS_DEBUG_STREAM("  - Upper: " << upper);
                jointMaximums(index) = upper;

                i++;
            }

            modelLink = modelLink->getParent();
        }

        //Build Solvers
        fk_solver     = std::make_unique<KDL::ChainFkSolverPos_recursive>(*chain);
        ik_solver_vel = std::make_unique<KDL::ChainIkSolverVel_wdls>(*chain);

        // Only the end effector position is needed, we do not care about it's rotation
        Eigen::MatrixXd matrix_Mx = Eigen::MatrixXd::Identity(6, 6);
        matrix_Mx(3, 3) = 0;
        matrix_Mx(4, 4) = 0;
        matrix_Mx(5, 5) = 0;
        ik_solver_vel->setWeightTS(matrix_Mx);
        ik_solver_pos = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(*chain, jointMinimums, jointMaximums, *fk_solver, *ik_solver_vel, 1000, 0.001f);
        //ik_solver_pos = std::make_unique<KDL::ChainIkSolverPos_NR>(*chain, *fk_solver, *ik_solver_vel, 1000, 0.01f);

        return true;
    }

    void Leg::tick(double now, double delta) {
        // noop
    }

    void Leg::setJointState(const sensor_msgs::JointState::ConstPtr &jointState) {
        for (auto &joint:joints) {
            joint.setJointState(jointState);
        }
    }

    void Leg::setGait(std::shared_ptr<Mildred::Gait> gait) {
        currentGait = std::move(gait);
    }

    tf2::Vector3 Leg::doGait() {
        tf2::Vector3 gaitTargetPosition = currentGait->walk(gaitConfig);
        tf2::Vector3 positionInLegFrame = frame * gaitTargetPosition;
        return positionInLegFrame;
    }

    std::pair<bool, tf2::Vector3> Leg::doFK() {
        KDL::JntArray     q_in(jointCount);
        for (unsigned int i = 0; i < jointCount; ++i) {
            q_in(i) = joints[i].currentPosition;
        }

        KDL::Frame p_out;
        int        fk_valid = fk_solver->JntToCart(q_in, p_out);
        if (fk_valid >= 0) {
            ROS_DEBUG("P_OUT: %f %f %f", p_out.p.x(), p_out.p.y(), p_out.p.z());
            return std::make_pair(true, tf2::Vector3(p_out.p.x(), p_out.p.y(), p_out.p.z()));
        } else {
            ROS_WARN("FK Solution not found for : %s, error: %d", name.c_str(), fk_valid);
            ROS_DEBUG("FK POS: %f %f %f", q_in(0), q_in(1), q_in(2));
            ROS_DEBUG("P_OUT: %f %f %f", p_out.p.x(), p_out.p.y(), p_out.p.z());
            return std::make_pair(false, tf2::Vector3(p_out.p.x(), p_out.p.y(), p_out.p.z()));
        }
    }

    bool Leg::doIK() {
        //if (initRun) {
        q_init(0) = 0.00;
        q_init(1) = -M_PI_2;
        q_init(2) = 3 * M_PI_4;
        //} else {
        //    //Get the current joint positions (our array is base->tip, IK works with tip->base)
        //    std::string       debug;
        //    for (unsigned int i = 0; i < jointCount; ++i) {
        //        debug += std::to_string(i) + ": " + std::to_string(joints[i].currentPosition) + " ";
        //        q_init(i) = joints[i].currentPosition;
        //    }
        //    ROS_DEBUG_STREAM(" -  CJP: " << debug);
        //}

        int ik_valid = ik_solver_pos->CartToJnt(q_init, KDL::Frame(KDL::Vector(targetPosition.x(), targetPosition.y(), targetPosition.z())), q_out);

        // Only when a solution is found it will be sent
        if (ik_valid >= 0) {
            for (unsigned int i = 0; i < jointCount; ++i) {
                joints[i].targetPosition = q_out(i);
            }
            initRun = false;

        } else {
            ROS_WARN("IK Solution not found for : %s, error: %d", name.c_str(), ik_valid);
            ROS_DEBUG("IK POS: %f %f %f", targetPosition.x(), targetPosition.y(), targetPosition.z());
            ROS_DEBUG("Q_OUT: %f %f %f", q_out(0), q_out(1), q_out(2));

            initRun = true;
        }

        return (ik_valid >= 0);
    }

    bool Leg::doIK(tf2::Vector3 target) {
        targetPosition = target;
        return doIK();
    }
}
