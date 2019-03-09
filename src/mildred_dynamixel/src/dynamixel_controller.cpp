#include "mildred_dynamixel/dynamixel_controller.h"

DynamixelController::DynamixelController(const std::string portName, const uint32_t baudRate)
    : nodeHandle(""),
      privateNodeHandle("~"),
      portName(portName),
      baudRate(baudRate),
      moving(false) {
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
    ROS_DEBUG("Loading joints configuration");

    std::string filePath = nodeHandle.param<std::string>("joints_config_file", "");
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

        dynamixelWorkbench->torqueOn(dxl.second);
    }

    return true;
}

/**
 * Not sure yet...
 */
bool DynamixelController::initControlItems() {
    ROS_DEBUG("Initializing control items");

    auto it = jointIds.begin();

    const ControlItem *goalPosition = dynamixelWorkbench->getItemInfo(it->second, "Goal_Position");
    if (goalPosition == nullptr) { return false; }

    const ControlItem *goalVelocity = dynamixelWorkbench->getItemInfo(it->second, "Goal_Velocity");
    if (goalVelocity == nullptr) { goalVelocity = dynamixelWorkbench->getItemInfo(it->second, "Moving_Speed"); }
    if (goalVelocity == nullptr) { return false; }

    const ControlItem *presentPosition = dynamixelWorkbench->getItemInfo(it->second, "Present_Position");
    if (presentPosition == nullptr) { return false; }

    const ControlItem *presentVelocity = dynamixelWorkbench->getItemInfo(it->second, "Present_Velocity");
    if (presentVelocity == nullptr) { presentVelocity = dynamixelWorkbench->getItemInfo(it->second, "Present_Speed"); }
    if (presentVelocity == nullptr) { return false; }

    const ControlItem *presentCurrent = dynamixelWorkbench->getItemInfo(it->second, "Present_Current");
    if (presentCurrent == nullptr) { presentCurrent = dynamixelWorkbench->getItemInfo(it->second, "Present_Load"); }
    if (presentCurrent == nullptr) { return false; }

    controlItems["Goal_Position"] = goalPosition;
    controlItems["Goal_Velocity"] = goalVelocity;

    controlItems["Present_Position"] = presentPosition;
    controlItems["Present_Velocity"] = presentVelocity;
    controlItems["Present_Current"] = presentCurrent;

    return true;
}

/**
 * Not sure either what this does
 */
bool DynamixelController::initSDKHandlers() {
    ROS_DEBUG("Initializing SDK handlers");

    const char *log = nullptr;

    if (!dynamixelWorkbench->addSyncWriteHandler(controlItems["Goal_Position"]->address, controlItems["Goal_Position"]->data_length, &log)) {
        ROS_ERROR("%s", log);
        return false;
    } else {
        ROS_INFO("%s", log);
    }

    if (!dynamixelWorkbench->addSyncWriteHandler(controlItems["Goal_Velocity"]->address, controlItems["Goal_Velocity"]->data_length, &log)) {
        ROS_ERROR("%s", log);
        return false;
    } else {
        ROS_INFO("%s", log);
    }

    if (dynamixelWorkbench->getProtocolVersion() == 2.0f) {
        uint16_t startAddress = std::min(controlItems["Present_Position"]->address, controlItems["Present_Current"]->address);
        uint16_t readLength = controlItems["Present_Position"]->data_length + controlItems["Present_Velocity"]->data_length + controlItems["Present_Current"]->data_length;
        if (!dynamixelWorkbench->addSyncReadHandler(startAddress, readLength, &log)) {
            ROS_ERROR("%s", log);
            return false;
        }
    }

    return true;
}

bool DynamixelController::getPresentPosition(std::vector<std::string> dynamixelNames) {
    const char *log = nullptr;
    bool result = true;

    int32_t position[dynamixelNames.size()];
    uint8_t idList[dynamixelNames.size()];
    uint8_t idCount = 0;

    for (auto const &name:dynamixelNames) {
        idList[idCount++] = jointIds[name];
    }

    WayPoint wp;

    if (dynamixelWorkbench->getProtocolVersion() == 2.0f) {
        if (!dynamixelWorkbench->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, idList, dynamixelNames.size(), &log)) {
            ROS_ERROR("%s", log);
            result = false;
        }

        if (dynamixelWorkbench->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, idList, idCount, controlItems["Present_Position"]->address, controlItems["Present_Position"]->data_length, position, &log)) {
            for (uint8_t index = 0; index < idCount; ++index) {
                wp.position = dynamixelWorkbench->convertValue2Radian(idList[index], position[index]);
                presentGoal.push_back(wp);
            }
        } else {
            ROS_ERROR("%s", log);
            result = false;
        }
    } else if (dynamixelWorkbench->getProtocolVersion() == 1.0f) {
        uint32_t read_position;
        for (auto const &dxl:jointIds) {
            if (!dynamixelWorkbench->readRegister((uint8_t) dxl.second, controlItems["Present_Position"]->address, controlItems["Present_Position"]->data_length, &read_position, &log)) {
                ROS_ERROR("%s", log);
                result = false;
            }

            wp.position = dynamixelWorkbench->convertValue2Radian((uint8_t) dxl.second, read_position);
            presentGoal.push_back(wp);
        }
    }

    return result;
}

void DynamixelController::initPublisher() {
    ROS_DEBUG("Initializing publishers");

    dynamixelStateListPublisher = privateNodeHandle.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 100);
    jointStatePublisher = privateNodeHandle.advertise<sensor_msgs::JointState>("joint_states", 100);
}

void DynamixelController::initSubscriber() {
    ROS_DEBUG("Initializing subscribers");
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
    dynamixelStateList.dynamixel_state.clear();

    int32_t current[jointIds.size()];
    int32_t velocity[jointIds.size()];
    int32_t position[jointIds.size()];

    uint8_t idList[jointIds.size()];
    uint8_t idCount = 0;

    for (auto const &dxl:jointIds) {
        dynamixelState[idCount].name = dxl.first;
        dynamixelState[idCount].id = (uint8_t) dxl.second;
        idList[idCount++] = (uint8_t) dxl.second;
    }
#ifndef DEBUG
    if (!moving) {
#endif
        if (dynamixelWorkbench->getProtocolVersion() == 2.0f) {
            if (!dynamixelWorkbench->syncRead(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, idList, jointIds.size(), &log)) {
                ROS_ERROR("%s", log);
            }

            if (!dynamixelWorkbench->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, idList, idCount, controlItems["Present_Current"]->address, controlItems["Present_Current"]->data_length, current, &log)) {
                ROS_ERROR("%s", log);
            }

            if (!dynamixelWorkbench->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, idList, idCount, controlItems["Present_Velocity"]->address, controlItems["Present_Velocity"]->data_length, velocity, &log)) {
                ROS_ERROR("%s", log);
            }

            if (!dynamixelWorkbench->getSyncReadData(SYNC_READ_HANDLER_FOR_PRESENT_POSITION_VELOCITY_CURRENT, idList, idCount, controlItems["Present_Position"]->address, controlItems["Present_Position"]->data_length, position, &log)) {
                ROS_ERROR("%s", log);
            }

            for (uint8_t index = 0; index < idCount; ++index) {
                dynamixelState[index].present_current = current[index];
                dynamixelState[index].present_velocity = velocity[index];
                dynamixelState[index].present_position = position[index];
                dynamixelStateList.dynamixel_state.push_back(dynamixelState[index]);
            }
        } else if (dynamixelWorkbench->getProtocolVersion() == 1.0f) {
            uint16_t dataLength = controlItems["Present_Position"]->data_length + controlItems["Present_Velocity"]->data_length + controlItems["Present_Current"]->data_length;
            uint32_t allData[dataLength];
            uint8_t dynamixelCount = 0;

            for (auto const &dxl:jointIds) {
                if (!dynamixelWorkbench->readRegister((uint8_t) dxl.second, controlItems["Present_Position"]->address, dataLength, allData, &log)) {
                    ROS_ERROR("%s", log);
                }

                dynamixelState[dynamixelCount].present_current = DXL_MAKEWORD(allData[4], allData[5]);
                dynamixelState[dynamixelCount].present_velocity = DXL_MAKEWORD(allData[2], allData[3]);
                dynamixelState[dynamixelCount].present_position = DXL_MAKEWORD(allData[0], allData[1]);

                dynamixelStateList.dynamixel_state.push_back(dynamixelState[dynamixelCount]);
                dynamixelCount++;
            }
        }
#ifndef DEBUG
    }
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
    dynamixelStateListPublisher.publish(dynamixelStateList);

    jointStateMessage.header.stamp = ros::Time::now();
    jointStateMessage.name.clear();
    jointStateMessage.position.clear();
    jointStateMessage.velocity.clear();
    jointStateMessage.effort.clear();

    uint8_t id_cnt = 0;

    for (auto const &dxl:jointIds) {
        double position = 0.0;
        double velocity = 0.0;
        double effort = 0.0;

        jointStateMessage.name.push_back(dxl.first);

        if (dynamixelWorkbench->getProtocolVersion() == 2.0f) {
            if (strcmp(dynamixelWorkbench->getModelName((uint8_t) dxl.second), "XL-320") == 0) {
                effort = dynamixelWorkbench->convertValue2Load((int16_t) dynamixelStateList.dynamixel_state[id_cnt].present_current);
            } else {
                effort = dynamixelWorkbench->convertValue2Current((int16_t) dynamixelStateList.dynamixel_state[id_cnt].present_current);
            }
        } else if (dynamixelWorkbench->getProtocolVersion() == 1.0f) {
            effort = dynamixelWorkbench->convertValue2Load((int16_t) dynamixelStateList.dynamixel_state[id_cnt].present_current);
        }

        velocity = dynamixelWorkbench->convertValue2Velocity((uint8_t) dxl.second, (int32_t) dynamixelStateList.dynamixel_state[id_cnt].present_velocity);
        position = dynamixelWorkbench->convertValue2Radian((uint8_t) dxl.second, (int32_t) dynamixelStateList.dynamixel_state[id_cnt].present_position);

        jointStateMessage.effort.push_back(effort);
        jointStateMessage.velocity.push_back(velocity);
        jointStateMessage.position.push_back(position);

        id_cnt++;
    }

    jointStatePublisher.publish(jointStateMessage);

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

    if (!dynamixelWorkbench->itemWrite(id, itemName.c_str(), value, &log)) {
        ROS_ERROR("%s", log);
        ROS_ERROR("Failed to write value[%d] on items[%s] to Dynamixel[ID : %d]", value, itemName.c_str(), id);
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
