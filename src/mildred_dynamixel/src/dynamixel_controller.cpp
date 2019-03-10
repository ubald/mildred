#include "mildred_dynamixel/dynamixel_controller.h"

DynamixelController::DynamixelController(const std::string portName, const uint32_t baudRate)
    : nodeHandle(""),
      privateNodeHandle("~"),
      portName(portName),
      baudRate(baudRate) {
    readPeriod = privateNodeHandle.param<double>("dxl_read_period", 0.010f);
    writePeriod = privateNodeHandle.param<double>("dxl_write_period", 0.010f);
    publishPeriod = privateNodeHandle.param<double>("publish_period", 0.010f);
    dynamixelWorkbench = new DynamixelWorkbench();
}

DynamixelController::~DynamixelController() = default;

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

    const char *log;
    if (!dynamixelWorkbench->init(portName.c_str(), baudRate, &log)) {
        ROS_ERROR("%s", log);
        ROS_ERROR("Please check USB port name");
        return false;
    }

    return true;
}

/**
 * Read joint configuration from the YAML file
 * @return
 */
bool DynamixelController::getDynamixelJointsInfo() {
    std::string filePath = privateNodeHandle.param<std::string>("joints_config_file", "");

    ROS_DEBUG("Loading joints configuration [%s]", filePath.c_str());

    YAML::Node config = YAML::LoadFile(filePath);
    if (config == nullptr) {
        return false;
    }

    for (const auto property: config["common"]) {
        auto field = property.first.as<std::string>();
        auto value = property.second.as<uint32_t>();
        commonJointConfig.emplace(std::make_pair(field, value));
    }

    //YAML::Node joints = config["joints"];
    for (const auto joint: config["joints"]) {
        const auto name = joint.first.as<std::string>();
        if (name.empty()) {
            continue;
        }

        ROS_DEBUG("%s:", name.c_str());

        auto newJointConfig = jointConfig.emplace(std::make_pair(name, std::unordered_map<std::string, uint32_t>()));

        for (const auto property: config["joints"][name]) {
            auto field = property.first.as<std::string>();
            auto value = property.second.as<uint32_t>();

            ROS_DEBUG("    %s: %d", field.c_str(), value);

            if (field == "ID") {
                jointIds[name] = value;
                continue;
            }

            newJointConfig.first->second.emplace(std::make_pair(field, value));
        }
    }

    return true;
}

/**
 * Ping all dynamixel actuators found in the YAML configuration
 */
bool DynamixelController::pingDynamixels() {
    ROS_DEBUG("Pinging actuators");

    const char *log;

    for (auto const &dxl:jointIds) {
        uint16_t modelNumber = 0;
        if (!dynamixelWorkbench->ping((uint8_t) dxl.second, &modelNumber, &log)) {
            ROS_ERROR("%s", log);
            ROS_ERROR("Can't find Dynamixel ID '%d'", dxl.second);
            return false;
        } else {
            ROS_INFO("Name: %s, ID: %d, Model Number: %d", dxl.first.c_str(), dxl.second, modelNumber);
        }
    }

    return true;
}

/**
 * Write the initial config from the YAML file to the dynamixel actuators
 */
bool DynamixelController::initDynamixels() {
    ROS_DEBUG("Initializing actuators");

    const char *log;

    for (auto const &dxl:jointIds) {
        ROS_DEBUG("Initializing Dynamixel[Name: %s, ID: %d]", dxl.first.c_str(), dxl.second);

        dynamixelWorkbench->torqueOff(dxl.second);

        // Common Config
        for (auto const &property:commonJointConfig) {
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
        for (auto const &property:jointConfig[dxl.first]) {
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

        // dynamixelWorkbench->torqueOn(dxl.second);
    }

    return true;
}

bool DynamixelController::initControlItems() {
    ROS_DEBUG("Initializing control items");

    // Just get the first and assume all the same model
    auto it = jointIds.begin();

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
    uint16_t readLength = currentPositionControl->data_length + currentVelocityControl->data_length + currentLoadControl->data_length;
    if (!dynamixelWorkbench->addSyncReadHandler(startAddress, readLength, &log)) {
        ROS_ERROR("%s", log);
        return false;
    }

    return true;
}

void DynamixelController::initPublisher() {
    ROS_DEBUG("Initializing publishers");
    dynamixelStatePublisher = nodeHandle.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 100);
    jointStatePublisher = nodeHandle.subscribe<sensor_msgs::JointState>("joint_states", 100);
}

void DynamixelController::initSubscriber() {
    ROS_DEBUG("Initializing subscribers");
    jointControlSubscriber = nodeHandle.advertise<sensor_msgs::JointState>("joint_states", 100);
}

void DynamixelController::initServices() {
    ROS_DEBUG("Initializing services");
    dynamixelCommandService = privateNodeHandle.advertiseService("dynamixel_command", &DynamixelController::dynamixelCommandMsgCallback, this);
}

void DynamixelController::readCallback(const ros::TimerEvent &) {
#ifdef DEBUG
    static double priv_read_secs = ros::Time::now().toSec();
#endif
    const char *log = nullptr;

    dynamixel_workbench_msgs::DynamixelState dynamixelState[jointIds.size()];
    dynamixelStates.dynamixel_state.clear();

    int32_t load[jointIds.size()];
    int32_t velocity[jointIds.size()];
    int32_t position[jointIds.size()];

    uint8_t idList[jointIds.size()];
    uint8_t idCount = 0;

    //for (auto const &dxl:jointIds) {
    //    jointState.name.push_back(dxl.first);
    //    idList[idCount++] = dxl.second;
    //}

    for (auto const& dxl:jointIds){
        dynamixelState[idCount].name = dxl.first;
        dynamixelState[idCount].id = dxl.second;
        idList[idCount++] = dxl.second;
    }

#ifndef DEBUG
    // if (!moving) {
#endif
        if (!dynamixelWorkbench->syncRead(CURRENT_STATE_SYNC_READ_HANDLER, idList, idCount, &log)) {
            ROS_ERROR("%s", log);
        }

        if (!dynamixelWorkbench->getSyncReadData(CURRENT_STATE_SYNC_READ_HANDLER, idList, idCount, currentLoadControl->address, currentLoadControl->data_length, load, &log)) {
            ROS_ERROR("%s", log);
        }

        if (!dynamixelWorkbench->getSyncReadData(CURRENT_STATE_SYNC_READ_HANDLER, idList, idCount, currentVelocityControl->address, currentVelocityControl->data_length, velocity, &log)) {
            ROS_ERROR("%s", log);
        }

        if (!dynamixelWorkbench->getSyncReadData(CURRENT_STATE_SYNC_READ_HANDLER, idList, idCount, currentPositionControl->address, currentPositionControl->data_length, position, &log)) {
            ROS_ERROR("%s", log);
        }

        //for (uint8_t index = 0; index < idCount; ++index) {
        //    jointState.effort.push_back(dynamixelWorkbench->convertValue2Current((int16_t) load[index])); // TODO: mA != N
        //    jointState.velocity.push_back(dynamixelWorkbench->convertValue2Velocity(idList[index], velocity[index]));
        //    jointState.position.push_back(dynamixelWorkbench->convertValue2Radian(idList[index], position[index]));
        //}

        for(uint8_t index = 0; index < idCount; index++){
            dynamixelState[index].present_current = (int16_t) load[index];
            dynamixelState[index].present_velocity = velocity[index];
            dynamixelState[index].present_position = position[index];
            dynamixelStates.dynamixel_state.push_back(dynamixelState[index]);
        }
#ifndef DEBUG
    // }
#endif

#ifdef DEBUG
    ROS_WARN("[readCallback] diff_secs : %f", ros::Time::now().toSec() - priv_read_secs);
    priv_read_secs = ros::Time::now().toSec();
#endif
}

void DynamixelController::publishCallback(const ros::TimerEvent &) {
#ifdef DEBUG
    static double priv_pub_secs =ros::Time::now().toSec();
#endif
    // TODO: Allow disabling
    dynamixelStatePublisher.publish(dynamixelStates);

    jointState.header.stamp = ros::Time::now();
    jointState.name.clear();
    jointState.position.clear();
    jointState.velocity.clear();
    jointState.effort.clear();

    uint8_t idCount = 0;
    for (auto const& dxl:jointIds) {
        jointState.name.push_back(dxl.first);
        jointState.effort.push_back(dynamixelWorkbench->convertValue2Current((int16_t) dynamixelStates.dynamixel_state[idCount].present_current)); // TODO: mA != N
        jointState.velocity.push_back(dynamixelWorkbench->convertValue2Velocity(dxl.second, dynamixelStates.dynamixel_state[idCount].present_velocity));
        jointState.position.push_back(dynamixelWorkbench->convertValue2Radian(dxl.second, dynamixelStates.dynamixel_state[idCount].present_position));
        idCount++;
    }

    jointStatePublisher.publish(jointState);
#ifdef DEBUG
    ROS_WARN("[publishCallback] diff_secs : %f", ros::Time::now().toSec() - priv_pub_secs);
    priv_pub_secs = ros::Time::now().toSec();
#endif
}

void DynamixelController::writeCallback(const ros::TimerEvent &) {
#ifdef DEBUG
    static double priv_pub_secs =ros::Time::now().toSec();
#endif
    //const char *log = nullptr;
    //
    ////uint8_t idList[dynamixels.size()];
    ////uint8_t idCount = 0;
    //
    //int32_t position[dynamixels.size()];
    //
    //static uint32_t pointCount    = 0;
    //static uint32_t positionCount = 0;
    //
    ////for (auto const &joint:jnt_tra_msg_->joint_names) {
    ////    idList[idCount] = (uint8_t) dynamixels[joint];
    ////    idCount++;
    ////}
    //
    //if (!moving) {
    //    for (uint8_t index = 0; index < idCount; ++index) {
    //        position[index] = dynamixelWorkbench->convertRadian2Value(idList[index], jnt_tra_msg_->points[pointCount].positions.at(index));
    //    }
    //
    //    if (!dynamixelWorkbench->syncWrite(SYNC_WRITE_HANDLER_FOR_GOAL_POSITION, idList, idCount, position, 1, &log)) {
    //        ROS_ERROR("%s", log);
    //    }
    //
    //    positionCount++;
    //    if (positionCount >= jnt_tra_msg_->points[pointCount].positions.size()) {
    //        pointCount++;
    //        positionCount = 0;
    //        if (pointCount >= jnt_tra_msg_->points.size()) {
    //            moving        = false;
    //            pointCount    = 0;
    //            positionCount = 0;
    //
    //            ROS_INFO("Complete Execution");
    //        }
    //    }
    //}

#ifdef DEBUG
    ROS_WARN("[writeCallback] diff_secs : %f", ros::Time::now().toSec() - priv_pub_secs);
    priv_pub_secs = ros::Time::now().toSec();
#endif
}

bool DynamixelController::dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req, dynamixel_workbench_msgs::DynamixelCommand::Response &res) {
    const char *log;

    uint8_t id = req.id;
    std::string itemName = req.addr_name;
    int32_t value = req.value;

    ROS_DEBUG("Writing value[%d] on items[%s] to Dynamixel[ID: %d]", value, itemName.c_str(), id);

    if (!dynamixelWorkbench->itemWrite(id, itemName.c_str(), value, &log)) {
        ROS_ERROR("%s", log);
        ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[ID: %d]", value, itemName.c_str(), id);
        res.comm_result = 0;
        return false;
    }

    res.comm_result = 1;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mildred_dynamixel_controller");
    ros::NodeHandle nodeHandle{};

    if (argc < 2) {
        ROS_ERROR("Please set '-port_name' and  '-baud_rate' arguments for connected Dynamixels");
        return 0;
    }

    std::string portName = argv[1]; // "/dev/ttyUSB0"
    uint32_t baudRate = atoi(argv[2]); // 57600

    DynamixelController dynamixelController{portName, baudRate};

    if (!dynamixelController.init()) {
        ROS_WARN("Failed to init, quitting");
        return 1;
    }

    ROS_INFO("Creating timers");

    ros::Timer readTimer = nodeHandle.createTimer(ros::Duration(dynamixelController.getReadPeriod()), &DynamixelController::readCallback, &dynamixelController);
    ros::Timer writeTimer = nodeHandle.createTimer(ros::Duration(dynamixelController.getWritePeriod()), &DynamixelController::writeCallback, &dynamixelController);
    ros::Timer publishTimer = nodeHandle.createTimer(ros::Duration(dynamixelController.getPublishPeriod()), &DynamixelController::publishCallback, &dynamixelController);

    ROS_INFO("Ready!");

    ros::spin();

    return 0;
}
