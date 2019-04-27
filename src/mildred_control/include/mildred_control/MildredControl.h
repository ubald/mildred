#pragma once

#define VIZ_DEBUG

#include <memory>

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include <mildred_core/mildred.h>
#include <mildred_core/MildredCommandMessage.h>
#include <mildred_core/MildredControlMessage.h>
#include <mildred_control/mildred/Body.h>
#include <mildred_control/mildred/states/ControlMachine.h>

namespace Mildred {
    class MildredControl {
      public:
        MildredControl();
        ~MildredControl() = default;

        bool init();
        void loop();

        std::unique_ptr<Mildred::Body> body{nullptr};

        void setActuatorState(bool on);
        bool publishJointPositions = false;

      private:
        ros::NodeHandle nodeHandle;

        ros::Publisher actuatorsStatePublisher;
        ros::Publisher mildredStatePublisher;

        ros::Subscriber             commandSubscriber;
        ros::Subscriber             controlSubscriber;
        ros::Subscriber             jointStatesSubscriber;
        ros::Publisher              targetJointPositionPublisher;
        std_msgs::Float64MultiArray targetJointPositionMessage;

        ControlMachine machine;

        double lastTime = 0;
        bool   hasState = false;

        void commandMessageCallback(const mildred_core::MildredCommandMessage::ConstPtr &controlMessage);
        void controlMessageCallback(const mildred_core::MildredControlMessage::ConstPtr &controlMessage);
        void jointsStatesCallback(const sensor_msgs::JointState::ConstPtr &jointStatesMessage);

        #ifdef VIZ_DEBUG
        ros::Publisher targetMarkersPublisher;
        #endif
    };

}