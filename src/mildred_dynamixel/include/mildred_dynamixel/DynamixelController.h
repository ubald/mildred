#pragma once

#include <memory>
#include <unordered_map>

#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

#include "mildred_core/ActuatorsStateMessage.h"

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define CURRENT_STATE_SYNC_READ_HANDLER 0

typedef struct {
    double offset     = 0.00f;
    double multiplier = 0.00f;
} JointConfig;

class DynamixelController {
    public:
        DynamixelController(std::string port, uint32_t baudRate);
        ~DynamixelController() = default;

        bool init();

        uint8_t getReadFrequency() { return readFrequency; }

        uint8_t getPublishFrequency() { return publishFrequency; }

        void readCallback(const ros::TimerEvent &);
        void publishCallback(const ros::TimerEvent &);

        void actuatorsStateCallback(const mildred_core::ActuatorsStateMessage::ConstPtr &actuatorsStateMessage);
        void dynamixelPositionControlCallback(const std_msgs::Float64MultiArray::ConstPtr &requestedPositionMessage);
        bool dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req, dynamixel_workbench_msgs::DynamixelCommand::Response &res);

    private:
        // ROS NodeHandle
        ros::NodeHandle nodeHandle;
        ros::NodeHandle privateNodeHandle;

        // ROS Parameters

        // ROS Topic Publisher
        ros::Publisher dynamixelStatePublisher;
        ros::Publisher jointStatePublisher;

        // ROS Topic Subscriber
        ros::Subscriber jointControlSubscriber;
        ros::Subscriber actuatorsStateSubscriber;

        // ROS Service Server
        ros::ServiceServer dynamixelCommandService;

        // ROS Service Client

        // Dynamixel Workbench Parameters
        std::string                         port;
        uint32_t                            baudRate;
        std::unique_ptr<DynamixelWorkbench> dynamixelWorkbench;

        std::vector<std::pair<std::string, uint8_t>>                               dxlIds{};
        std::unordered_map<std::string, uint32_t>                                  commonDxlConfig{};
        std::unordered_map<std::string, std::unordered_map<std::string, uint32_t>> dxlConfig{};
        std::unordered_map<uint8_t, JointConfig>                                   jointConfigs{};

        bool publishDynamixelState = false;
        bool torqueEnabled         = false;

        const ControlItem *goalPositionControl;
        const ControlItem *currentPositionControl;
        const ControlItem *currentVelocityControl;
        const ControlItem *currentLoadControl;

        dynamixel_workbench_msgs::DynamixelStateList dynamixelStates{};
        sensor_msgs::JointState                      jointState{};

        uint8_t                     readFrequency;
        uint8_t                     writeFrequency;
        uint8_t                     publishFrequency;
        std::unique_ptr<ros::Timer> readTimer;
        std::unique_ptr<ros::Timer> publishTimer;

        bool initWorkbench();
        bool getDynamixelJointsInfo();
        bool pingDynamixels();
        bool initDynamixels();
        bool initControlItems();
        bool initSDKHandlers();
        void initPublisher();
        void initSubscriber();
        void initServices();
};