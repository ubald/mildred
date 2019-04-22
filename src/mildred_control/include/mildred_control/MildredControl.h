#pragma once

#define VIZ_DEBUG

#include <memory>

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include "mildred_control/fsm/Machine.h"
#include "mildred_core/mildred.h"
#include "mildred_core/RemoteControlMessage.h"

#include "mildred_control/mildred/Body.h"

namespace Mildred {

    class MildredControl {
        public:
            MildredControl();
            ~MildredControl() = default;

            bool init();
            void loop();

            ros::Publisher                 actuatorsStatePublisher;
            std::unique_ptr<Mildred::Body> body{nullptr};

            bool publishJointPositions = false;

        private:
            ros::NodeHandle nodeHandle;

            ros::Subscriber             controlSubscriber;
            ros::Subscriber             jointStatesSubscriber;
            ros::Publisher              targetJointPositionPublisher;
            std_msgs::Float64MultiArray targetJointPositionMessage;

            Mildred::Machine machine;

            double lastTime = 0;
            bool hasState = false;

            void controlMessageCallback(const mildred_core::RemoteControlMessage::ConstPtr &controlMessage);
            void jointsStatesCallback(const sensor_msgs::JointState::ConstPtr &jointStatesMessage);

            #ifdef VIZ_DEBUG
            ros::Publisher targetMarkersPublisher;
            #endif
    };

}