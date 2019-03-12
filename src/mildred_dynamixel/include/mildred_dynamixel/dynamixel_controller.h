#pragma once

#include <iostream>
#include <unordered_map>

#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/joint_state.hpp"
#include <mildred_dynamixel/msg/dynamixel_state_list.hpp>
#include <mildred_dynamixel/srv/dynamixel_command.hpp>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define CURRENT_STATE_SYNC_READ_HANDLER 0

// #define DEBUG

class DynamixelController : public rclcpp::Node {
private:
    // ROS NodeHandle

    // ROS Parameters

    // ROS Topic Publisher
    std::shared_ptr<rclcpp::Publisher<mildred_dynamixel::msg::DynamixelStateList>> dynamixelStatePublisher;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> jointStatePublisher;

    // ROS Topic Subscriber

    // ROS Service Server
    std::shared_ptr<rclcpp::Service<mildred_dynamixel::srv::DynamixelCommand>> dynamixelCommandService;

    // ROS Service Client

    // Dynamixel Workbench Parameters
    std::string port;
    uint32_t baudRate;
    std::unique_ptr<DynamixelWorkbench> dynamixelWorkbench;

    std::map<std::string, uint8_t> jointIds{};
    std::unordered_map<std::string, uint32_t> commonJointConfig{};
    std::unordered_map<std::string, std::unordered_map<std::string, uint32_t>> jointConfig{};

    const ControlItem *goalPositionControl;
    const ControlItem *currentPositionControl;
    const ControlItem *currentVelocityControl;
    const ControlItem *currentLoadControl;

    mildred_dynamixel::msg::DynamixelStateList dynamixelStates{};
    sensor_msgs::msg::JointState jointState{};

    uint8_t readFrequency;
    uint8_t writeFrequency;
    uint8_t publishFrequency;
    std::shared_ptr<rclcpp::TimerBase> readTimer;
    std::shared_ptr<rclcpp::TimerBase> writeTimer;
    std::shared_ptr<rclcpp::TimerBase> publishTimer;

    bool initWorkbench();
    bool getDynamixelJointsInfo();
    bool pingDynamixels();
    bool initDynamixels();
    bool initControlItems();
    bool initSDKHandlers();
    void initPublisher();
    void initSubscriber();
    void initServices();

public:
    DynamixelController();
    ~DynamixelController();

    bool init();

    void readCallback();
    void writeCallback();
    void publishCallback();

    void dynamixelCommandMsgCallback(
        std::shared_ptr<rmw_request_id_t> request_header,
        std::shared_ptr<mildred_dynamixel::srv::DynamixelCommand::Request> request,
        std::shared_ptr<mildred_dynamixel::srv::DynamixelCommand::Response> response
    );
};