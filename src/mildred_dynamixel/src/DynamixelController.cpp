#include "mildred_dynamixel/DynamixelController.h"

DynamixelController::DynamixelController(const std::string port, const uint32_t baudRate)
    : nodeHandle(""),
      privateNodeHandle("~"),
      port(port),
      baudRate(baudRate),
      dynamixelWorkbench(std::make_unique<DynamixelWorkbench>()),
      readFrequency(static_cast<uint8_t>(privateNodeHandle.param("read_frequency", 60))),
      publishFrequency(static_cast<uint8_t>(privateNodeHandle.param("publish_frequency", 60))) {}

bool DynamixelController::init() {
    if (!initWorkbench()) {
        ROS_ERROR("Please check USB port name");
        return false;
    }

    if (!getDynamixelJointsInfo()) {
        ROS_ERROR("Please check YAML file");
        return false;
    }

    if (!pingDynamixels()) {
        ROS_ERROR("Please check Dynamixel ID or BaudRate");
        return false;
    }

    if (!initDynamixels()) {
        ROS_ERROR("Please check control table (http://emanual.robotis.com/#control-table)");
        return false;
    }

    if (!initControlItems()) {
        ROS_ERROR("Please check control items");
        return false;
    }

    if (!initSDKHandlers()) {
        ROS_ERROR("Failed to set Dynamixel SDK Handler");
        return false;
    }

    initPublisher();
    initSubscriber();
    initServices();

    return true;
}

bool DynamixelController::initWorkbench() {
    ROS_DEBUG("Initializing workbench");
    ROS_DEBUG("Port: %s Baudrate: %d", port.c_str(), baudRate);

    const char *log;
    if (!dynamixelWorkbench->init(port.c_str(), baudRate, &log)) {
        ROS_ERROR("%s", log);
        ROS_ERROR("Please check USB port name");
        return false;
    }

    return true;
}

bool DynamixelController::getDynamixelJointsInfo() {
    std::string filePath = privateNodeHandle.param<std::string>("joints_config_file", "");

    ROS_DEBUG("Loading joints configuration [%s]", filePath.c_str());

    YAML::Node config = YAML::LoadFile(filePath);
    if (config == nullptr) {
        return false;
    }

    for (const auto property: config["common"]["dxl"]) {
        auto field = property.first.as<std::string>();
        auto value = property.second.as<uint32_t>();
        commonDxlConfig.emplace(field, value);
    }

    for (const auto joint: config["joints"]) {
        const auto name = joint["name"].as<std::string>();
        if (name.empty()) {
            ROS_ERROR("Name is required in YAML joint config");
            continue;
        }

        const auto dxlId = joint["id"].as<uint32_t>();
        dxlIds.emplace_back(name, dxlId);

        JointConfig jointConfig;
        jointConfig.offset     = joint["offset"].as<double>();
        jointConfig.multiplier = joint["multiplier"].as<double>();
        jointConfigs.emplace(dxlId, jointConfig);

        ROS_DEBUG_STREAM(name << ": id(" << dxlId << "):");

        auto newJointConfig = dxlConfig.emplace(name, std::unordered_map<std::string, uint32_t>());

        for (const auto property: joint["dxl"]) {
            auto field = property.first.as<std::string>();
            auto value = property.second.as<uint32_t>();
            ROS_DEBUG_STREAM("    " << field << ": " << value);
            newJointConfig.first->second.emplace(field, value);
        }
    }

    return true;
}

bool DynamixelController::pingDynamixels() {
    ROS_DEBUG("Pinging actuators");

    const char *log;

    for (auto const &dxl:dxlIds) {
        uint16_t modelNumber = 0;
        if (!dynamixelWorkbench->ping(dxl.second, &modelNumber, &log)) {
            ROS_ERROR("%s", log);
            ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
            return false;
        } else {
            ROS_INFO("Name: %s, ID: %d, Model Number: %d", dxl.first.c_str(), dxl.second, modelNumber);
        }
    }

    return true;
}

bool DynamixelController::initDynamixels() {
    ROS_DEBUG("Initializing actuators");

    const char *log;

    for (auto const &dxl:dxlIds) {
        ROS_DEBUG("Initializing Dynamixel[Name: %s, ID: %d]", dxl.first.c_str(), dxl.second);

        dynamixelWorkbench->torqueOff(dxl.second);

        // Common Config
        for (auto const &property:commonDxlConfig) {
            if (property.first == "ID" || property.first == "Baud_Rate") {
                continue;
            }

            ROS_DEBUG("Writing [%s=%d] to Dynamixel[Name: %s, ID: %d]", property.first.c_str(), property.second, dxl.first.c_str(), dxl.second);
            if (!dynamixelWorkbench->itemWrite(dxl.second, property.first.c_str(), property.second, &log)) {
                ROS_ERROR("%s", log);
                ROS_ERROR("Failed to write [%s=%d] to Dynamixel[Name: %s, ID: %d]", property.first.c_str(), property.second, dxl.first.c_str(), dxl.second);
                return false;
            }
        }

        // Custom Config
        for (auto const &property:dxlConfig[dxl.first]) {
            if (property.first == "ID" || property.first == "Baud_Rate") {
                continue;
            }

            ROS_DEBUG("Setting [%s=%d] to Dynamixel[Name: %s, ID: %d]", property.first.c_str(), property.second, dxl.first.c_str(), dxl.second);
            if (!dynamixelWorkbench->itemWrite(dxl.second, property.first.c_str(), property.second, &log)) {
                ROS_ERROR("%s", log);
                ROS_ERROR("Failed to write [%s=%d] to Dynamixel[Name: %s, ID: %d]", property.first.c_str(), property.second, dxl.first.c_str(), dxl.second);
                return false;
            }
        }

        if (torqueEnabled) {
            dynamixelWorkbench->torqueOn(dxl.second);
        }
    }

    return true;
}

bool DynamixelController::initControlItems() {
    ROS_DEBUG("Initializing control items");

    // Just get the first and assume all the same model
    auto it = dxlIds.begin();

    goalPositionControl = dynamixelWorkbench->getItemInfo(it->second, "Goal_Position");
    if (goalPositionControl == nullptr) { return false; }

    currentPositionControl = dynamixelWorkbench->getItemInfo(it->second, "Present_Position");
    if (currentPositionControl == nullptr) { return false; }

    currentVelocityControl = dynamixelWorkbench->getItemInfo(it->second, "Present_Velocity");
    if (currentVelocityControl == nullptr) { currentVelocityControl = dynamixelWorkbench->getItemInfo(it->second, "Present_Speed"); }
    if (currentVelocityControl == nullptr) { return false; }

    currentLoadControl = dynamixelWorkbench->getItemInfo(it->second, "Present_Current");
    if (currentLoadControl == nullptr) { currentLoadControl = dynamixelWorkbench->getItemInfo(it->second, "Present_Load"); }
    if (currentLoadControl == nullptr) { return false; }

    return true;
}

bool DynamixelController::initSDKHandlers() {
    ROS_DEBUG("Initializing SDK handlers");

    const char *log = nullptr;

    if (!dynamixelWorkbench->addSyncWriteHandler(goalPositionControl->address, goalPositionControl->data_length, &log)) {
        ROS_ERROR("%s", log);
        return false;
    } else {
        ROS_INFO("%s", log);
    }

    uint16_t startAddress = std::min(currentPositionControl->address, currentLoadControl->address);
    uint16_t readLength   = currentPositionControl->data_length + currentVelocityControl->data_length + currentLoadControl->data_length;
    if (!dynamixelWorkbench->addSyncReadHandler(startAddress, readLength, &log)) {
        ROS_ERROR("%s", log);
        return false;
    } else {
        ROS_INFO("%s", log);
    }

    return true;
}

void DynamixelController::initPublisher() {
    ROS_DEBUG("Initializing publishers");
    dynamixelStatePublisher = nodeHandle.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 100);
    jointStatePublisher     = nodeHandle.advertise<sensor_msgs::JointState>("joint_states", 100);
}

void DynamixelController::initSubscriber() {
    ROS_DEBUG("Initializing subscribers");
    jointControlSubscriber   = nodeHandle.subscribe("joint_position_controller/command", 1, &DynamixelController::dynamixelPositionControlCallback, this);
    actuatorsStateSubscriber = nodeHandle.subscribe("actuators_state", 100, &DynamixelController::actuatorsStateCallback, this);
}

void DynamixelController::initServices() {
    ROS_DEBUG("Initializing services");
    dynamixelCommandService = privateNodeHandle.advertiseService("dynamixel_command", &DynamixelController::dynamixelCommandMsgCallback, this);
}

void DynamixelController::readCallback(const ros::TimerEvent &) {
    const char *log = nullptr;

    dynamixel_workbench_msgs::DynamixelState dynamixelState[dxlIds.size()];
    dynamixelStates.dynamixel_state.clear();

    int32_t load[dxlIds.size()];
    int32_t velocity[dxlIds.size()];
    int32_t position[dxlIds.size()];

    uint8_t idList[dxlIds.size()];
    uint8_t idCount = 0;

    for (auto const &dxl:dxlIds) {
        dynamixelState[idCount].name = dxl.first;
        dynamixelState[idCount].id   = dxl.second;
        // TODO: Store that list
        idList[idCount++] = dxl.second;
    }

    if (!dynamixelWorkbench->syncRead(CURRENT_STATE_SYNC_READ_HANDLER, idList, idCount, &log)) {
        ROS_ERROR("Sync Read Error: %s", log);
        return;
    }

    if (!dynamixelWorkbench->getSyncReadData(CURRENT_STATE_SYNC_READ_HANDLER, idList, idCount, currentLoadControl->address, currentLoadControl->data_length, load, &log)) {
        ROS_ERROR("Load Data Error: %s", log);
        return;
    }

    if (!dynamixelWorkbench->getSyncReadData(CURRENT_STATE_SYNC_READ_HANDLER, idList, idCount, currentVelocityControl->address, currentVelocityControl->data_length, velocity, &log)) {
        ROS_ERROR("Velocity Data Error: %s", log);
        return;
    }

    if (!dynamixelWorkbench->getSyncReadData(CURRENT_STATE_SYNC_READ_HANDLER, idList, idCount, currentPositionControl->address, currentPositionControl->data_length, position, &log)) {
        ROS_ERROR("Position Data Error: %s", log);
        return;
    }

    for (uint8_t index = 0; index < idCount; index++) {
        dynamixelState[index].present_position = position[index];
        dynamixelState[index].present_velocity = velocity[index];
        dynamixelState[index].present_current  = static_cast<int16_t>(load[index]);
        dynamixelStates.dynamixel_state.push_back(dynamixelState[index]);
    }
}

void DynamixelController::publishCallback(const ros::TimerEvent &) {
    if (publishDynamixelState) {
        dynamixelStatePublisher.publish(dynamixelStates);
    }

    jointState.header.stamp = ros::Time::now();
    jointState.name.clear();
    jointState.position.clear();
    jointState.velocity.clear();
    jointState.effort.clear();

    uint8_t idCount = 0;

    for (auto const &dxl:dxlIds) {
        JointConfig jointConfig = jointConfigs.at(dxl.second);

        auto position = static_cast<int32_t>(dynamixelStates.dynamixel_state[idCount].present_position * jointConfig.multiplier + jointConfig.offset);

        jointState.name.push_back(dxl.first);
        jointState.position.push_back(dynamixelWorkbench->convertValue2Radian(dxl.second, position));
        jointState.velocity.push_back(dynamixelWorkbench->convertValue2Velocity(dxl.second, dynamixelStates.dynamixel_state[idCount].present_velocity));
        jointState.effort.push_back(dynamixelWorkbench->convertValue2Load((int16_t) dynamixelStates.dynamixel_state[idCount].present_current)); // TODO: mA != N
        idCount++;
    }

    jointStatePublisher.publish(jointState);
}

void DynamixelController::actuatorsStateCallback(const mildred_core::ActuatorsStateMessage::ConstPtr &actuatorsStateMessage) {
    ROS_INFO_STREAM("Turning torque " << (actuatorsStateMessage->state > 0 ? "ON" : "OFF") << " on all joints");
    for (auto const &dxl:dxlIds) {
        if (actuatorsStateMessage->state > 0) {
            dynamixelWorkbench->torqueOn(dxl.second);
        } else {
            dynamixelWorkbench->torqueOff(dxl.second);
        }
    }
}

void DynamixelController::dynamixelPositionControlCallback(const std_msgs::Float64MultiArray::ConstPtr &requestedPositionMessage) {
    const char *log = nullptr;

    auto idCount = static_cast<uint8_t>(dxlIds.size());
    if (idCount > requestedPositionMessage->data.size()) {
        ROS_WARN("Received less positions than the number of joints");
        return;
    }

    uint8_t index = 0;
    uint8_t idList[idCount];
    int32_t position[idCount];

    for (auto const &dxl:dxlIds) {
        JointConfig jointConfig = jointConfigs.at(dxl.second);
        idList[index]   = dxl.second;
        position[index] = dynamixelWorkbench->convertRadian2Value(dxl.second, static_cast<float>((requestedPositionMessage->data[index] - jointConfig.offset) / jointConfig.multiplier));
        index++;
    }

    if (!dynamixelWorkbench->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, idList, idCount, position, 1, &log)) {
        ROS_ERROR("Error writing position: %s", log);
    }
}

bool DynamixelController::dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req, dynamixel_workbench_msgs::DynamixelCommand::Response &res) {
    const char *log;

    uint8_t     id       = req.id;
    std::string itemName = req.addr_name;
    int32_t     value    = req.value;

    ROS_DEBUG("Writing value[%d] on items[%s] to Dynamixel[ID: %d]", value, itemName.c_str(), id);

    if (dynamixelWorkbench->itemWrite(id, itemName.c_str(), value, &log)) {
        res.comm_result = 1;
        return true;
    } else {
        ROS_ERROR("%s", log);
        ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[ID: %d]", value, itemName.c_str(), id);
        res.comm_result = 0;
        return false;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mildred_dynamixel_controller");
    ros::NodeHandle nodeHandle{};

    if (argc < 2) {
        ROS_ERROR("Please set port' and baudRate arguments for connected Dynamixels");
        return 0;
    }

    std::string port     = argv[1]; // "/dev/ttyUSB0"
    uint32_t    baudRate = atoi(argv[2]); // 57600

    DynamixelController dynamixelController{port, baudRate};

    if (!dynamixelController.init()) {
        ROS_WARN("Failed to init, quitting");
        return 1;
    }

    ros::Timer readTimer    = nodeHandle.createTimer(ros::Duration(1.0f / (double) dynamixelController.getReadFrequency()), &DynamixelController::readCallback, &dynamixelController);
    ros::Timer publishTimer = nodeHandle.createTimer(ros::Duration(1.0f / (double) dynamixelController.getPublishFrequency()), &DynamixelController::publishCallback, &dynamixelController);

    ROS_INFO("Ready!");

    ros::spin();

    return 0;
}
