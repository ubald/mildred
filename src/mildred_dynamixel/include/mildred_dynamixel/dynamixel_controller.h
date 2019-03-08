#pragma once

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
#define SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT 0

// #define DEBUG

typedef struct {
    std::string itemName;
    uint32_t    value;
} DynamixelConfigValue;

class DynamixelController {
private:
    // ROS NodeHandle
    ros::NodeHandle nodeHandle;
    ros::NodeHandle privateNodeHandle;

    // ROS Parameters

    // ROS Topic Publisher
    ros::Publisher dynamixelStateListPublisher;
    ros::Publisher jointStatePublisher;

    // ROS Topic Subscriber

    // ROS Service Server
    ros::ServiceServer dynamixelCommandService;

    // ROS Service Client

    // Dynamixel Workbench Parameters
    std::string        portName;
    uint32_t           baudRate;
    DynamixelWorkbench *dynamixelWorkbench;

    std::map<std::string, uint32_t> dynamixels{};

    std::vector<std::pair<std::string, DynamixelConfigValue>> dynamixelJointConfigs{};

    std::map<std::string, const ControlItem *>   controlItems;
    dynamixel_workbench_msgs::DynamixelStateList dynamixelStateList;
    sensor_msgs::JointState                      jointStateMessage;
    std::vector<WayPoint>                        presentGoal;

    double readPeriod;
    double writePeriod;
    double publishPeriod;

    bool moving;

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
    DynamixelController(std::string portName, uint32_t baudRate);
    ~DynamixelController();

    bool init();

    bool getPresentPosition(std::vector<std::string> dynamixelNames);

    double getReadPeriod() { return readPeriod; }
    double getWritePeriod() { return writePeriod; }
    double getPublishPeriod() { return publishPeriod; }

    void readCallback(const ros::TimerEvent &);
    void writeCallback(const ros::TimerEvent &);
    void publishCallback(const ros::TimerEvent &);

    bool dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req,dynamixel_workbench_msgs::DynamixelCommand::Response &res);
};