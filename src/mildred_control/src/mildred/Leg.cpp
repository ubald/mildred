#include <utility>

#include "mildred_control/mildred/Leg.h"

namespace Mildred {
    Leg::Leg(unsigned int index) :
        index(index),
        name("leg_" + std::to_string(index)) {
        gaitConfig.index = index;
    }

    bool Leg::setup(std::shared_ptr<urdf::Model> model, std::string tip) {
        ROS_DEBUG("Setting up leg %d \"%s\" with tip \"%s\"", index, name.c_str(), tip.c_str());

        // Count joints and check that chain isn't broken
        jointCount = 0;
        std::string linkName = tip + "_" + std::to_string(index);
        auto        link     = model->getLink(linkName);
        while (link && link->parent_joint) {
            if (link->parent_joint->type != urdf::Joint::UNKNOWN && link->parent_joint->type != urdf::Joint::FIXED) {
                jointCount++;
            }

            link = link->getParent();
        }

        ROS_DEBUG("  Found %d joints", jointCount);
        jointMinimums.resize(jointCount);
        jointMaximums.resize(jointCount);

        // Re-traverse the chain and save limits
        link = model->getLink(linkName);
        unsigned int i = 0;
        while (link && link->parent_joint) {
            const auto joint = link->parent_joint;
            ROS_DEBUG("  Inspecting joint %s, type: %u, parent: %s", joint->name.c_str(), joint->type, joint->parent_link_name.c_str());

            if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
                unsigned int jointIndex = jointCount - i - 1;
                joints[jointIndex].name           = joint->name;
                joints[jointIndex].targetPosition = 0.00f;

                double lower, upper;
                if (joint->type != urdf::Joint::CONTINUOUS) {
                    lower = joint->limits->lower;
                    upper = joint->limits->upper;
                } else {
                    lower = -M_PI;
                    upper = M_PI;
                }

                jointMinimums[jointIndex] = lower;
                jointMaximums[jointIndex] = upper;

                ROS_DEBUG("    Joint %d (%s) limits: %f - %f", jointIndex, joint->name.c_str(), lower, upper);

                i++;
            }

            auto position = joint->parent_to_joint_origin_transform.position;

            // All of this is custom per our robot definition, we<re not using KDL or any other IK library, so we
            // just calculate the various required values manually from what we already know about the robot.
            if (joint->parent_link_name == "coxa_" + std::to_string(index)) {
                // For IK purposes, we only consider lengths in the 2d axis they move in
                coxaLength = tf2::Vector3(position.x, position.y, 0 /* position.z */).length();
            } else if (joint->parent_link_name == "femur_" + std::to_string(index)) {
                femurLength = tf2::Vector3(position.x, position.y, position.z).length();
            } else if (joint->parent_link_name == "tibia_" + std::to_string(index)) {
                tibiaLength = tf2::Vector3(position.x, position.y, position.z).length();
            } else if (joint->parent_link_name == "thorax") {
                // Find our position/rotation in the body
                double r, p, y;
                joint->parent_to_joint_origin_transform.rotation.getRPY(r, p, y);
                frame.getOrigin().setValue(position.x, position.y, position.z);
                frame.getBasis().setRPY(r, p, y);
                gaitConfig.alpha = y;
            }

            link = link->getParent();
        }

        return true;
    }

    void Leg::tick(double now, double delta) {
        // noop
    }

    void Leg::setJointState(const sensor_msgs::JointState::ConstPtr &jointState) {
        for (auto &joint:joints) {
            joint.setJointState(jointState);
        }

        auto fk = doFK();
        if (!fk.first) {
            ROS_ERROR_STREAM("Failed to compute FK for leg " << name << " to update current position.");
            return;
        }

        currentPosition = fk.second;
    }

    void Leg::setGait(std::shared_ptr<Mildred::Gait> gait) {
        currentGait = std::move(gait);
    }

    tf2::Vector3 Leg::doGait() {
        tf2::Vector3 gaitTargetPosition = currentGait->walk(gaitConfig);
        //tf2::Vector3 positionInLegFrame = frame * gaitTargetPosition;
        return gaitTargetPosition;//positionInLegFrame;
    }

    std::pair<bool, tf2::Vector3> Leg::doFK() {
        return std::make_pair(true, tf2::Vector3(0, 0, 0));

        //KDL::JntArray     q_in(jointCount);
        //for (unsigned int i = 0; i < jointCount; ++i) {
        //    q_in(i) = joints[i].currentPosition;
        //}
        //
        //KDL::Frame p_out;
        //int        fk_valid = fk_solver->JntToCart(q_in, p_out);
        //if (fk_valid >= 0) {
        //    //ROS_DEBUG("P_OUT: %f %f %f", p_out.p.x(), p_out.p.y(), p_out.p.z());
        //    return std::make_pair(true, tf2::Vector3(p_out.p.x(), p_out.p.y(), p_out.p.z()));
        //} else {
        //    ROS_WARN("FK Solution not found for : %s, error: %d", name.c_str(), fk_valid);
        //    ROS_DEBUG("FK POS: %f %f %f", q_in(0), q_in(1), q_in(2));
        //    ROS_DEBUG("P_OUT: %f %f %f", p_out.p.x(), p_out.p.y(), p_out.p.z());
        //    return std::make_pair(false, tf2::Vector3(p_out.p.x(), p_out.p.y(), p_out.p.z()));
        //}
    }

    bool Leg::doIK(tf2::Vector3 target) {
        targetPosition = target;

        double footPosX = targetPosition.x();
        double footPosY = targetPosition.y();
        double footPosZ = targetPosition.z();

        //ROS_WARN_STREAM(footPosX << " " << footPosY << " " << footPosZ);

        // Length between the Root and Foot Position ...Pythagorean theorem
        double hipToFoot = sqrt(pow(footPosX, 2) + pow(footPosY, 2)) - coxaLength;
        double hipToFoot_abs = std::abs(hipToFoot);
        if (hipToFoot_abs > femurLength + tibiaLength || hipToFoot_abs < tibiaLength - femurLength) {
            ROS_FATAL("IK Solver cannot solve a foot position that is not within leg reach!");
            ROS_FATAL_STREAM(
                "Leg=" << std::to_string(index) <<
                " X=" << std::to_string(footPosX) <<
                " Y=" << std::to_string(footPosY) <<
                " Z=" << std::to_string(footPosZ) <<
                " hip_to_foot=" << std::to_string(hipToFoot) <<
                " min_limit=" << std::to_string(tibiaLength - femurLength) <<
                " max_limit=" << std::to_string(femurLength + tibiaLength)
            );
            ROS_FATAL("Shutting down so configuration can be fixed!!!");
            //ros::shutdown();
        }

        // Length from hip joint to foot
        double lhf = sqrt(pow(hipToFoot, 2) + pow(footPosZ, 2));
        double lhf_sqr = pow(lhf, 2);

        // Length of the sides of the triangle formed by the femur and tibia joints.
        double femurLength_sqr = pow(femurLength, 2);
        double tibiaLength_sqr = pow(tibiaLength, 2);

        // We are using the law of cosines on the triangle formed by the femur and tibia joints.
        double angle_b = acos((femurLength_sqr - tibiaLength_sqr + lhf_sqr) / (2.0 * femurLength * lhf));
        double angle_c = acos((femurLength_sqr + tibiaLength_sqr - lhf_sqr) / (2.0 * femurLength * tibiaLength));

        // Angle of line between the femur and foot joints with respect to feet_pos_z.
        double theta = atan2(hipToFoot, footPosZ);
        //ROS_ERROR_STREAM(std::to_string(footPosZ) << " " << std::to_string(hipToFoot) << " " << std::to_string(theta) << " " << std::to_string(angle_b));

        joints[0].targetPosition = M_PI_2 - atan2(footPosX, footPosY);
        //joints[1].targetPosition = M_PI_2 - (theta + angle_b);
        joints[1].targetPosition = (theta - angle_b) - M_PI_2;
        joints[2].targetPosition = M_PI - angle_c;
        //ROS_DEBUG_STREAM(joints[0].targetPosition << " " << joints[1].targetPosition << " " << joints[2].targetPosition);

        return true;

        ////if (initRun) {
        //q_init(0) = 0.00;
        //q_init(1) = -M_PI_2;
        //q_init(2) = 3 * M_PI_4;
        ////} else {
        ////    //Get the current joint positions (our array is base->tip, IK works with tip->base)
        ////    std::string       debug;
        ////    for (unsigned int i = 0; i < jointCount; ++i) {
        ////        debug += std::to_string(i) + ": " + std::to_string(joints[i].currentPosition) + " ";
        ////        q_init(i) = joints[i].currentPosition;
        ////    }
        ////    ROS_DEBUG_STREAM(" -  CJP: " << debug);
        ////}
        //
        //KDL::Frame p_in(KDL::Vector(targetPosition.x(), targetPosition.y(), targetPosition.z()));
        //int ik_valid = ik_solver_pos->CartToJnt(q_init, p_in, q_out);
        //
        //// Only when a solution is found it will be sent
        //if (ik_valid >= 0) {
        //    for (unsigned int i = 0; i < jointCount; ++i) {
        //        joints[i].targetPosition = q_out(i);
        //    }
        //    initRun = false;
        //
        //} else {
        //    ROS_WARN("IK Solution not found for : %s, error: %d", name.c_str(), ik_valid);
        //    ROS_DEBUG("IK POS: %f %f %f", targetPosition.x(), targetPosition.y(), targetPosition.z());
        //    ROS_DEBUG("Q_OUT: %f %f %f", q_out(0), q_out(1), q_out(2));
        //
        //    initRun = true;
        //}
        //
        //return (ik_valid >= 0);
    }
}
