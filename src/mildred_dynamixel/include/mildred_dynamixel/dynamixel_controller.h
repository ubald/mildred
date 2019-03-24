#pragma once

#include <memory>
#include <unordered_map>

#include <ros/ros.h>

#include <yaml-cpp/yaml.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <dynamixel_workbench_toolbox/dynamixel_workbench.h>
#include <dynamixel_workbench_msgs/DynamixelStateList.h>
#include <dynamixel_workbench_msgs/DynamixelCommand.h>

#include <dynamixel_workbench_controllers/trajectory_generator.h>

// SYNC_WRITE_HANDLER
#define SYNC_WRITE_HANDLER_FOR_GOAL_POSITION 0
#define SYNC_WRITE_HANDLER_FOR_GOAL_VELOCITY 1

// SYNC_READ_HANDLER(Only for Protocol 2.0)
#define CURRENT_STATE_SYNC_READ_HANDLER 0

// #define DEBUG

class DynamixelController {
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

    // ROS Service Server
    ros::ServiceServer dynamixelCommandService;

    // ROS Service Client

    // Dynamixel Workbench Parameters
    std::string port;
    uint32_t baudRate;
    std::unique_ptr<DynamixelWorkbench> dynamixelWorkbench;

    std::map<std::string, uint8_t> jointIds{};
    std::unordered_map<std::string, uint32_t> commonJointConfig{};
    std::unordered_map<std::string, std::unordered_map<std::string, uint32_t>> jointConfig{};

    const ControlItem * goalPositionControl;
    const ControlItem * currentPositionControl;
    const ControlItem * currentVelocityControl;
    const ControlItem * currentLoadControl;

    dynamixel_workbench_msgs::DynamixelStateList dynamixelStates{};
    sensor_msgs::JointState jointState{};

    uint8_t readFrequency;
    uint8_t writeFrequency;
    uint8_t publishFrequency;
    std::unique_ptr<ros::Timer> readTimer;
    std::unique_ptr<ros::Timer> writeTimer;
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

public:
    DynamixelController(std::string port, uint32_t baudRate);
    ~DynamixelController();

    bool init();

    uint8_t getReadFrequency() { return readFrequency; }
    uint8_t getWriteFrequency() { return writeFrequency; }
    uint8_t getPublishFrequency() { return publishFrequency; }

    void readCallback(const ros::TimerEvent &);
    void writeCallback(const ros::TimerEvent &);
    void publishCallback(const ros::TimerEvent &);

    bool dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req, dynamixel_workbench_msgs::DynamixelCommand::Response &res);
};