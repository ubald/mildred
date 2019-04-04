#include <utility>

#include "mildred_control/frame/Leg.h"

namespace Mildred {
    Leg::Leg(unsigned int index) :
        name("leg_" + std::to_string(index)) {
        gaitConfig.index = index;
    }

    bool Leg::setup(std::shared_ptr<urdf::Model> model, std::unique_ptr<KDL::Chain> legChain, std::string tip) {
        //Setup pid
        //lastPIDTime = ros::Time::now().sec + (ros::Time::now().nsec * 1e-9);
        //pidX.initPid(0.05, 0.00, 0.00, 0, 0);
        //pidY.initPid(0.05, 0.00, 0.00, 0, 0);
        //pidZ.initPid(0.05, 0.00, 0.00, 0, 0);

        //Name our leg, for easier identification in debug info
        //name = chain->segments[0].getName();

        chain = std::move(legChain);

        ROS_INFO("Setting up leg %s starting at %s", name.c_str(), chain->segments[0].getName().c_str());

        // Compose root frame and leg/hip frame
        // This is highly dependent on our specific URDF model design
        frame = chain->segments[0].getFrameToTip() * chain->segments[1].getFrameToTip();

        //Gait configuration
        double r, p, y;
        frame.M.GetRPY(r, p, y);
        gaitConfig.alpha = y;

        //Initialize the total number of joints
        jointCount = 0;

        // Get the joints limits

        // Count joints and check that chain isn't broken
        auto modelLink = model->getLink(tip);
        while (modelLink && modelLink->parent_joint) {
            //if (!modelLink->parent_joint) {
                //ROS_ERROR("Link %s has no parent joint", modelLink->name.c_str());
                //return false;
            //}

            if (modelLink->parent_joint->type != urdf::Joint::UNKNOWN && modelLink->parent_joint->type != urdf::Joint::FIXED) {
                jointCount++;
            }

            //modelLink = model->getLink(modelLink->getParent()->name);
            modelLink = modelLink->getParent();
        }

        ROS_INFO("Found %d joints", jointCount);
        KDL::JntArray jointMinimums(jointCount);
        KDL::JntArray jointMaximums(jointCount);

        //Re-traverse the chain and save limits
        modelLink = model->getLink(tip);
        unsigned int i = 0;
        while (modelLink && modelLink->parent_joint) {
            const auto modelJoint = modelLink->parent_joint;
            if (modelJoint->type != urdf::Joint::UNKNOWN && modelJoint->type != urdf::Joint::FIXED) {
                ROS_INFO("Getting bounds for joint: %s", modelJoint->name.c_str());

                //Save name to our joint object
                joints[i].name = modelJoint->name;
                //TODO: Save actual joint pointer, create new UWalker::Joint here...

                //Get limits
                //TODO: Modify our solver to support a UWalker::Joint array instead and save these limits to our joint
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

                i++;
            }

            modelLink = modelLink->getParent();
        }

        //Build Solvers
        fk_solver     = std::make_unique<KDL::ChainFkSolverPos_recursive>(*chain);
        ik_solver_vel = std::make_unique<KDL::ChainIkSolverVel_wdls>(*chain);

        //Only the end effector position is needed - Task Space
        //NOTE: I don't know the fuck this does but without it it can't solve...
        //Eigen::MatrixXd matrix_Mx = Eigen::MatrixXd::Identity(6, 6);
        //matrix_Mx(3, 3) = 0;
        //matrix_Mx(4, 4) = 0;
        //matrix_Mx(5, 5) = 0;
        //ik_solver_vel->setWeightTS(matrix_Mx);
        ik_solver_pos = std::make_unique<KDL::ChainIkSolverPos_NR>(*chain, *fk_solver, *ik_solver_vel, 1000, 0.001f);

        //Resize our q_init array
        q_init.resize(jointCount);
        q_out.resize(jointCount);

        //Assign initialization values, if IK fails on startup look here...
        /*init_run = true;
         for ( unsigned int j = 0; j < jointCount; j++)
         {
         q_init(j) = M_PI / 2;
         }*/

        //Default joint init values for IK
        joints[0].targetPosition = 0.00f;
        joints[1].targetPosition = 0.00f; // -M_PI_2;
        joints[2].targetPosition = 0.00f;// 3 * M_PI_4;

        return true;
    }

    void Leg::setJointState(const sensor_msgs::JointState::ConstPtr& jointState) {
        for (auto &joint:joints) {
            joint.setJointState(jointState);
        }
    }

    void Leg::setGait(std::shared_ptr<Mildred::Gait> gait) {
        //TODO: Wait for the right moment to switch gait
        //TODO: Implement state machine with transitions
        currentGait = std::move(gait);
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
        targetPosition = target;
        ROS_ERROR_STREAM( "x:" << target.x() << " y:" << target.y() << " z:" << target.z() );
        //lastPIDTime    = now;

        //Get the current joint positions (our array is base->tip, IK works with tip->base)
        for (unsigned int i = 0; i < JOINT_COUNT; i++) {
            std::cout << joints[i].currentPosition << std::endl;
            q_init((JOINT_COUNT - 1) - i) = joints[i].currentPosition;
        }

        int ik_valid = ik_solver_pos->CartToJnt(q_init, KDL::Frame(target), q_out);
        // Only when a solution is found it will be sent
        if (ik_valid >= 0) {
            init_run            = false;
            for (unsigned int i = 0; i < jointCount; i++) {
                joints[i].targetPosition = q_out(i);
            }
        } else {
            ROS_WARN("Leg::doIK() IK Solution not found for : %s, error: %d", name.c_str(), ik_valid);
            ROS_DEBUG("Leg::doIK() IK POS: %f %f %f", target.x(), target.y(), target.z());
            ROS_DEBUG("Leg::doIK() Q_OUT: %f %f %f", q_out(0), q_out(1), q_out(2));
            init_run = true;

            //for (unsigned int i = 0; i < jointCount; i++) {
            //    joints[i].targetPosition = q_out(i);
            //}
        }

        return (ik_valid >= 0);
    }
}
