#include <mildred_control/MildredControl.h>

#include <mildred_core/ActuatorsStateMessage.h>
#include <mildred_core/MildredCommandMessage.h>
#include <mildred_core/MildredStateMessage.h>

#include "mildred/states/IdleState.h"
#include "mildred/states/StandingState.h"
#include "mildred/states/SittingState.h"
#include "mildred/states/StartingState.h"
#include "mildred/states/WalkingState.h"

namespace Mildred {

    MildredControl::MildredControl() :
        nodeHandle(),
        body(std::make_unique<Mildred::Body>()) {

        commandSubscriber            = nodeHandle.subscribe("command", 1, &MildredControl::commandMessageCallback, this);
        controlSubscriber            = nodeHandle.subscribe("control", 1, &MildredControl::controlMessageCallback, this);
        jointStatesSubscriber        = nodeHandle.subscribe("joint_states", 1, &MildredControl::jointsStatesCallback, this);
        targetJointPositionPublisher = nodeHandle.advertise<std_msgs::Float64MultiArray>("joint_position_controller/command", 1);
        actuatorsStatePublisher      = nodeHandle.advertise<mildred_core::ActuatorsStateMessage>("actuators_state", 1, true);
        mildredStatePublisher        = nodeHandle.advertise<mildred_core::MildredStateMessage>("mildred_state", 1, true);

        #ifdef VIZ_DEBUG
        targetMarkersPublisher = nodeHandle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        #endif

        auto starting = std::make_shared<StartingState>(this);
        auto idle     = std::make_shared<IdleState>(this);
        auto sitting  = std::make_shared<SittingState>(this);
        auto standing = std::make_shared<StandingState>(this);
        auto walking  = std::make_shared<WalkingState>(this);

        machine.onStateChange = [this](std::shared_ptr<ControlState> state) {
            ROS_INFO_STREAM("Publishing state changed to " << state->name());
            mildred_core::MildredStateMessage message;
            message.state = static_cast<decltype(message.state)>(state->id());
            mildredStatePublisher.publish(message);
        };

        machine.addState(starting, true);
        machine.addState(idle);
        machine.addState(sitting);
        machine.addState(standing);
        machine.addState(walking);

        machine.addTransition<Enable>(starting, idle);

        machine.addTransition<Sit>(idle, sitting);

        machine.addTransition<Stand>(sitting, standing);
        machine.addTransition<Standby>(sitting, idle);
        machine.addTransition<Ragdoll>(sitting, idle);

        machine.addTransition<Sit>(standing, sitting);
        machine.addTransition<Walk>(standing, walking);
        machine.addTransition<Ragdoll>(standing, idle);

        machine.addTransition<Stand>(walking, standing);
        machine.addTransition<Ragdoll>(walking, idle);
    }

    bool MildredControl::init() {
        auto model = std::make_shared<urdf::Model>();
        if (!model->initParam("robot_description")) {
            ROS_ERROR("Could not generate robot model");
            return false;
        }

        ROS_INFO("Loaded %s model", model->getName().c_str());

        if (!body->setup(model, LEG_COUNT, "foot_frame")) {
            ROS_ERROR("Failed to setup body");
            return false;
        }

        targetJointPositionMessage.data.resize(LEG_COUNT * DOF);

        return true;
    }

    void MildredControl::loop() {
        double now   = ros::Time::now().toSec();
        double delta = lastTime > 0 ? now - lastTime : 0.00f;
        lastTime = now;

        body->tick(now, delta);
        machine.tick(now, delta);

        #ifdef VIZ_DEBUG
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp    = ros::Time::now();
        marker.ns              = nodeHandle.getNamespace();
        marker.id              = 0;
        marker.type            = visualization_msgs::Marker::POINTS;
        marker.action          = visualization_msgs::Marker::ADD;
        marker.scale.x         = 0.03;
        marker.scale.y         = 0.03;
        //marker.scale.z         = 0.03;
        marker.color.a         = 1.0;
        marker.color.r         = 1.0;
        marker.color.g         = 0.5;
        marker.color.b         = 0.0;
        #endif

        uint8_t count = 0;


        for (auto const &leg:body->legs) {
            for (auto const &joint:leg->joints) {
                targetJointPositionMessage.data[count] = joint.targetPosition;
                count++;
            }

            #ifdef VIZ_DEBUG
            tf2::Vector3 target = leg->frame * leg->targetPosition;
            if (target.length() == std::numeric_limits<double>::infinity()) {
                continue;
            }

            geometry_msgs::Point point;
            point.x = target.x();
            point.y = target.y();
            point.z = target.z();
            marker.points.push_back(point);
            #endif
        }

        if (publishJointPositions) {
            targetJointPositionPublisher.publish(targetJointPositionMessage);
        }

        #ifdef VIZ_DEBUG
        targetMarkersPublisher.publish(marker);
        #endif
    }

    void MildredControl::setActuatorState(bool on) {
        mildred_core::ActuatorsStateMessage message;
        message.state = static_cast<decltype(message.state)>(on ? 1 : 0);
        actuatorsStatePublisher.publish(message);
        publishJointPositions = on;
    }

    void MildredControl::commandMessageCallback(const mildred_core::MildredCommandMessage::ConstPtr &commandMessage) {
        auto command = static_cast<MildredCommand>(commandMessage->command);
        switch (command) {
            case MildredCommand::Ragdoll:
                machine.handleEvent(Ragdoll());
                break;
            case MildredCommand::Standby:
                machine.handleEvent(Standby());
                break;
            case MildredCommand::Sit:
                machine.handleEvent(Sit());
                break;
            case MildredCommand::Stand:
                machine.handleEvent(Stand());
                break;
            case MildredCommand::Walk:
                machine.handleEvent(Walk());
                break;
        }
    }

    void MildredControl::controlMessageCallback(const mildred_core::MildredControlMessage::ConstPtr &controlMessage) {
        machine.handleControl(controlMessage);
    }

    void MildredControl::jointsStatesCallback(const sensor_msgs::JointState::ConstPtr &jointStatesMessage) {
        if (!body) {
            ROS_WARN("Received joint states before having a body");
            return;
        }

        body->setJointState(jointStatesMessage);

        if (!hasState) {
            ROS_INFO("Got first joint state message, switching to idle");
            hasState = true;
            machine.handleEvent(Enable());
        }
    }

}