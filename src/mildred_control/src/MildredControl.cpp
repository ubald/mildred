#include "mildred_control/MildredControl.h"

namespace Mildred {

    MildredControl::MildredControl() :
        nodeHandle(),
        body(std::make_unique<Mildred::Body>()){

        controlSubscriber     = nodeHandle.subscribe("/mildred/control", 1, &MildredControl::controlMessageCallback, this);
        jointStatesSubscriber = nodeHandle.subscribe("/mildred/joint_states", 1, &MildredControl::jointsStatesCallback, this);

        #ifdef VIZ_DEBUG
        targetMarkersPublisher = nodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        #endif
    }

    bool MildredControl::init() {
        auto model = std::make_shared<urdf::Model>();
        if (!model->initParam("robot_description")) {
            ROS_ERROR("mildred_control::main() Could not generate robot model");
            return false;
        }

        ROS_INFO("Loaded %s model", model->getName().c_str());

        // TODO: Put names in config
        if (!body->setup(model, "foot_frame")) {
            ROS_ERROR("mildred_control::main() Failed to setup body");
            return false;
        }

        return true;
    }

    void MildredControl::loop() {
        // TODO Stuff unrelated to the actual walk

        // NOTE Manual leg control

        // NOTE Gait switching

        /**
         * Tick the body loop
         * Gait calculations and body/leg IK all happens happens in this call.
         */
        body->tick();

#ifdef VIZ_DEBUG
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp    = ros::Time::now();
        marker.ns              = "mildred";
        marker.id              = 0;
        marker.type            = visualization_msgs::Marker::POINTS;
        marker.action          = visualization_msgs::Marker::ADD;
        marker.scale.x         = 0.005;
        marker.scale.y         = 0.005;
        marker.scale.z         = 0.005;
        marker.color.a         = 1.0;
        marker.color.r         = 1.0;
        marker.color.g         = 0.5;
        marker.color.b         = 0.0;
#endif

        //Send the joint positions values to the actuators
        //Mildred::LegsJointsPositions legsJointsPositionsMsg;
        for (auto const &leg:body->legs) {
            //for (unsigned int j = 0; j < body->leg[i]->JOINT_COUNT; j++) {
            //    legsJointsPositionsMsg.leg[i].joint[j] = body->leg[i]->joint[j].targetPosition;
            //}

#ifdef VIZ_DEBUG
            //Add point to marker points
            geometry_msgs::Point point;
            std::cout << leg->name << " " << leg->targetPosition.x() << " " << leg->targetPosition.y() << " " << leg->targetPosition.z() << std::endl;
            point.x = leg->targetPosition.x();
            point.y = leg->targetPosition.y();
            point.z = leg->targetPosition.z();
            marker.points.push_back(point);
#endif
        }

        //TODO Joint limits (not necessarily in here)

        //Publish final joints positions
        //legsJointsPositionsPublisher.publish(legsJointsPositionsMsg);

#ifdef VIZ_DEBUG
        targetMarkersPublisher.publish(marker);
#endif
    }

    void MildredControl::controlMessageCallback(const mildred_core::RemoteControlMessage::ConstPtr &controlMessage) {
        body->velocity = KDL::Vector2(controlMessage->velocity.x, controlMessage->velocity.y);
        body->frame    = KDL::Frame(
            KDL::Rotation::RPY(
                controlMessage->rotation.y * (M_PI / 8), //TODO: Dynamically adjust maximum rotation
                controlMessage->rotation.x * (M_PI / 8), //TODO: Dynamically adjust maximum rotation
                controlMessage->rotation.z * (M_PI / 8)  //TODO: Dynamically adjust maximum rotation
            ),
            KDL::Vector(
                controlMessage->position.x / 10.00, //TODO: Dynamically adjust maximum position
                controlMessage->position.y / 10.00, //TODO: Dynamically adjust maximum position
                controlMessage->position.z / 10.00  //TODO: Dynamically adjust maximum position
            )
        );
    }

    /**
     * Callback getting the joint's current position from the jointStates messages
     */
    void MildredControl::jointsStatesCallback(const sensor_msgs::JointState::ConstPtr &jointStatesMessage) {
        if (!body) {
            ROS_WARN("Received joint states before having a body");
        }
        body->setJointState(jointStatesMessage);
    }

}