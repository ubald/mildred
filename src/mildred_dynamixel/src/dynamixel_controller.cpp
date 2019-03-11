#include "mildred_dynamixel/dynamixel_controller.h"

DynamixelController::DynamixelController()
    : rclcpp::Node("dynamixel_controller") {
    this->get_parameter<std::string>("port", portName);
    this->get_parameter<uint32_t>("baudrate", baudRate);

    std::cout << portName << '/' << baudRate << std::endl;

    this->get_parameter_or<double>("dxl_read_period", readPeriod, 0.010f);
    this->get_parameter_or<double>("dxl_write_period", writePeriod, 0.010f);
    this->get_parameter_or<double>("publish_period", publishPeriod, 0.010f);

    dynamixelWorkbench = std::make_unique<DynamixelWorkbench>();
}

DynamixelController::~DynamixelController() = default;

bool DynamixelController::init() {
    if (!initWorkbench()) {
        RCLCPP_ERROR(this->get_logger(), "Please check USB port name");
        return false;
    }

    if (!getDynamixelJointsInfo()) {
        RCLCPP_ERROR(this->get_logger(), "Please check YAML file");
        return false;
    }

    if (!pingDynamixels()) {
        RCLCPP_ERROR(this->get_logger(), "Please check Dynamixel ID or BaudRate");
        return false;
    }

    if (!initDynamixels()) {
        RCLCPP_ERROR(this->get_logger(), "Please check control table (http://emanual.robotis.com/#control-table)");
        return false;
    }

    if (!initControlItems()) {
        RCLCPP_ERROR(this->get_logger(), "Please check control items");
        return false;
    }

    if (!initSDKHandlers()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set Dynamixel SDK Handler");
        return false;
    }

    initPublisher();
    initSubscriber();
    initServices();


    RCLCPP_INFO(this->get_logger(), "Creating timers");

    //readTimer = this->create_wall_timer(std::chrono::duration<double>(readPeriod), std::bind(&DynamixelController::readCallback, this), nullptr);
    //writeTimer = this->create_wall_timer(writePeriod, std::bind(&DynamixelController::writeCallback, this), nullptr);
    //publishTimer = this->create_wall_timer(publishPeriod, std::bind(&DynamixelController::publishCallback, this), nullptr);

    return true;
}

bool DynamixelController::initWorkbench() {
    RCLCPP_DEBUG(this->get_logger(), "Initializing workbench");

    const char *log;
    if (!dynamixelWorkbench->init(portName.c_str(), baudRate, &log)) {
        RCLCPP_ERROR(this->get_logger(), "%s", log);
        RCLCPP_ERROR(this->get_logger(), "Please check USB port name");
        return false;
    }

    return true;
}

/**
 * Read joint configuration from the YAML file
 * @return
 */
bool DynamixelController::getDynamixelJointsInfo() {
    std::string filePath;
    this->get_parameter_or<std::string>("joints_config_file", filePath, "");

    RCLCPP_DEBUG(this->get_logger(), "Loading joints configuration [%s]", filePath.c_str());

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

        RCLCPP_DEBUG(this->get_logger(), "%s:", name.c_str());

        auto newJointConfig = jointConfig.emplace(std::make_pair(name, std::unordered_map<std::string, uint32_t>()));

        for (const auto property: config["joints"][name]) {
            auto field = property.first.as<std::string>();
            auto value = property.second.as<uint32_t>();

            RCLCPP_DEBUG(this->get_logger(), "    %s: %d", field.c_str(), value);

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
    RCLCPP_DEBUG(this->get_logger(), "Pinging actuators");

    const char *log;

    for (auto const &dxl:jointIds) {
        uint16_t modelNumber = 0;
        if (!dynamixelWorkbench->ping((uint8_t) dxl.second, &modelNumber, &log)) {
            RCLCPP_ERROR(this->get_logger(), "%s", log);
            RCLCPP_ERROR(this->get_logger(), "Can't find Dynamixel ID '%d'", dxl.second);
            return false;
        } else {
            RCLCPP_INFO(this->get_logger(), "Name: %s, ID: %d, Model Number: %d", dxl.first.c_str(), dxl.second, modelNumber);
        }
    }

    return true;
}

/**
 * Write the initial config from the YAML file to the dynamixel actuators
 */
bool DynamixelController::initDynamixels() {
    RCLCPP_DEBUG(this->get_logger(), "Initializing actuators");

    const char *log;

    for (auto const &dxl:jointIds) {
        RCLCPP_DEBUG(this->get_logger(), "Initializing Dynamixel[Name: %s, ID: %d]", dxl.first.c_str(), dxl.second);

        dynamixelWorkbench->torqueOff(dxl.second);

        // Common Config
        for (auto const &property:commonJointConfig) {
            if (property.first == "ID" || property.first == "Baud_Rate") {
                continue;
            }

            RCLCPP_DEBUG(this->get_logger(), "Writing [%s=%d] to Dynamixel[Name: %s, ID: %d]", property.first.c_str(), property.second, dxl.first.c_str(), dxl.second);
            if (!dynamixelWorkbench->itemWrite(dxl.second, property.first.c_str(), property.second, &log)) {
                RCLCPP_ERROR(this->get_logger(), "%s", log);
                RCLCPP_ERROR(this->get_logger(), "Failed to write [%s=%d] to Dynamixel[Name: %s, ID: %d]", property.first.c_str(), property.second, dxl.first.c_str(), dxl.second);
                return false;
            }
        }

        // Custom Config
        for (auto const &property:jointConfig[dxl.first]) {
            if (property.first == "ID" || property.first == "Baud_Rate") {
                continue;
            }

            RCLCPP_DEBUG(this->get_logger(), "Setting [%s=%d] to Dynamixel[Name: %s, ID: %d]", property.first.c_str(), property.second, dxl.first.c_str(), dxl.second);
            if (!dynamixelWorkbench->itemWrite(dxl.second, property.first.c_str(), property.second, &log)) {
                RCLCPP_ERROR(this->get_logger(), "%s", log);
                RCLCPP_ERROR(this->get_logger(), "Failed to write [%s=%d] to Dynamixel[Name: %s, ID: %d]", property.first.c_str(), property.second, dxl.first.c_str(), dxl.second);
                return false;
            }
        }

        // dynamixelWorkbench->torqueOn(dxl.second);
    }

    return true;
}

bool DynamixelController::initControlItems() {
    RCLCPP_DEBUG(this->get_logger(), "Initializing control items");

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
    RCLCPP_DEBUG(this->get_logger(), "Initializing SDK handlers");

    const char *log = nullptr;

    if (!dynamixelWorkbench->addSyncWriteHandler(goalPositionControl->address, goalPositionControl->data_length, &log)) {
        RCLCPP_ERROR(this->get_logger(), "%s", log);
        return false;
    } else {
        RCLCPP_INFO(this->get_logger(), "%s", log);
    }

    uint16_t startAddress = std::min(currentPositionControl->address, currentLoadControl->address);
    uint16_t readLength = currentPositionControl->data_length + currentVelocityControl->data_length + currentLoadControl->data_length;
    if (!dynamixelWorkbench->addSyncReadHandler(startAddress, readLength, &log)) {
        RCLCPP_ERROR(this->get_logger(), "%s", log);
        return false;
    }

    return true;
}

void DynamixelController::initPublisher() {
    RCLCPP_DEBUG(this->get_logger(), "Initializing publishers");
    dynamixelStatePublisher = this->create_publisher<mildred_dynamixel::msg::DynamixelStateList>("dynamixel_state");
    jointStatePublisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states");
}

void DynamixelController::initSubscriber() {
    RCLCPP_DEBUG(this->get_logger(), "Initializing subscribers");

}

void DynamixelController::initServices() {
    RCLCPP_DEBUG(this->get_logger(), "Initializing services");
    dynamixelCommandService = this->create_service<mildred_dynamixel::srv::DynamixelCommand>(
        "dynamixel_command",
        std::bind(&DynamixelController::dynamixelCommandMsgCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void DynamixelController::readCallback() {
#ifdef DEBUG
    static double priv_read_secs = rclcpp::Time::now().toSec();
#endif
    const char *log = nullptr;

    mildred_dynamixel::msg::DynamixelState dynamixelState[jointIds.size()];
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

    for (auto const &dxl:jointIds) {
        dynamixelState[idCount].name = dxl.first;
        dynamixelState[idCount].id = dxl.second;
        idList[idCount++] = dxl.second;
    }

#ifndef DEBUG
    // if (!moving) {
#endif
    if (!dynamixelWorkbench->syncRead(CURRENT_STATE_SYNC_READ_HANDLER, idList, idCount, &log)) {
        RCLCPP_ERROR(this->get_logger(), "%s", log);
    }

    if (!dynamixelWorkbench->getSyncReadData(CURRENT_STATE_SYNC_READ_HANDLER, idList, idCount, currentLoadControl->address, currentLoadControl->data_length, load, &log)) {
        RCLCPP_ERROR(this->get_logger(), "%s", log);
    }

    if (!dynamixelWorkbench->getSyncReadData(CURRENT_STATE_SYNC_READ_HANDLER, idList, idCount, currentVelocityControl->address, currentVelocityControl->data_length, velocity, &log)) {
        RCLCPP_ERROR(this->get_logger(), "%s", log);
    }

    if (!dynamixelWorkbench->getSyncReadData(CURRENT_STATE_SYNC_READ_HANDLER, idList, idCount, currentPositionControl->address, currentPositionControl->data_length, position, &log)) {
        RCLCPP_ERROR(this->get_logger(), "%s", log);
    }

    //for (uint8_t index = 0; index < idCount; ++index) {
    //    jointState.effort.push_back(dynamixelWorkbench->convertValue2Current((int16_t) load[index])); // TODO: mA != N
    //    jointState.velocity.push_back(dynamixelWorkbench->convertValue2Velocity(idList[index], velocity[index]));
    //    jointState.position.push_back(dynamixelWorkbench->convertValue2Radian(idList[index], position[index]));
    //}

    for (uint8_t index = 0; index < idCount; index++) {
        dynamixelState[index].current_position = position[index];
        dynamixelState[index].current_velocity = velocity[index];
        dynamixelState[index].current_load = (int16_t) load[index];
        dynamixelStates.dynamixel_state.push_back(dynamixelState[index]);
    }
#ifndef DEBUG
    // }
#endif

#ifdef DEBUG
    RCLCPP_WARN("[readCallback] diff_secs : %f", rclcpp::Time::now().toSec() - priv_read_secs);
    priv_read_secs = rclcpp::Time::now().toSec();
#endif
}

void DynamixelController::publishCallback() {
#ifdef DEBUG
    static double priv_pub_secs =rclcpp::Time::now().toSec();
#endif
    // TODO: Allow disabling
    dynamixelStatePublisher->publish(dynamixelStates);

    jointState.header.stamp = this->now();
    jointState.name.clear();
    jointState.position.clear();
    jointState.velocity.clear();
    jointState.effort.clear();

    uint8_t idCount = 0;
    for (auto const &dxl:jointIds) {
        jointState.name.push_back(dxl.first);
        jointState.position.push_back(dynamixelWorkbench->convertValue2Radian(dxl.second, dynamixelStates.dynamixel_state[idCount].current_position));
        jointState.velocity.push_back(dynamixelWorkbench->convertValue2Velocity(dxl.second, dynamixelStates.dynamixel_state[idCount].current_velocity));
        jointState.effort.push_back(dynamixelWorkbench->convertValue2Current((int16_t) dynamixelStates.dynamixel_state[idCount].current_load)); // TODO: mA != N
        idCount++;
    }

    jointStatePublisher->publish(jointState);
#ifdef DEBUG
    RCLCPP_WARN("[publishCallback] diff_secs : %f", rclcpp::Time::now().toSec() - priv_pub_secs);
    priv_pub_secs = rclcpp::Time::now().toSec();
#endif
}

void DynamixelController::writeCallback() {
#ifdef DEBUG
    static double priv_pub_secs =rclcpp::Time::now().toSec();
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
    //        RCLCPP_ERROR("%s", log);
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
    //            RCLCPP_INFO("Complete Execution");
    //        }
    //    }
    //}

#ifdef DEBUG
    RCLCPP_WARN("[writeCallback] diff_secs : %f", rclcpp::Time::now().toSec() - priv_pub_secs);
    priv_pub_secs = rclcpp::Time::now().toSec();
#endif
}

void DynamixelController::dynamixelCommandMsgCallback(
    const std::shared_ptr<rmw_request_id_t> /*request_header*/,
    const std::shared_ptr<mildred_dynamixel::srv::DynamixelCommand::Request> request,
    const std::shared_ptr<mildred_dynamixel::srv::DynamixelCommand::Response> response
) {
    const char *log;

    uint8_t id = request->id;
    std::string itemName = request->addr_name;
    int32_t value = request->value;

    RCLCPP_DEBUG(this->get_logger(), "Writing value[%d] on items[%s] to Dynamixel[ID: %d]", value, itemName.c_str(), id);

    if (dynamixelWorkbench->itemWrite(id, itemName.c_str(), value, &log)) {
        response->comm_result = 1;
    } else {
        RCLCPP_ERROR(this->get_logger(), "%s", log);
        RCLCPP_ERROR(this->get_logger(), "Failed to write value[%d] on items[%s] to Dynamixel[ID: %d]", value, itemName.c_str(), id);
        response->comm_result = 0;
    }

}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DynamixelController>();
    if (!node->init()) {
        std::cerr << "Failed to init, quitting" << std::endl;
        return 1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();

    node = nullptr;

    return 0;
}
