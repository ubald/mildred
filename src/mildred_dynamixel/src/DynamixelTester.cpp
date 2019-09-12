#include <mildred_dynamixel/DynamixelTester.h>

#include <math.h>
#include <std_msgs/Float64MultiArray.h>
#include <mildred_core/ActuatorsStateMessage.h>
#include <sensor_msgs/JointState.h>

bool initialized = false;
std_msgs::Float64MultiArray targetJointPositionMessage;

void jointsStatesCallback(const sensor_msgs::JointState::ConstPtr &jointStatesMessage){
    if ( !initialized) {
        targetJointPositionMessage.data.resize(jointStatesMessage->position.size());

        for (unsigned int i = 0; i < jointStatesMessage->position.size(); ++i) {
            targetJointPositionMessage.data[i] = jointStatesMessage->position[i];
        }

        initialized = true;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mildred_dynamixel_tester");

    ros::NodeHandle nodeHandle{};
    ros::Publisher targetJointPositionPublisher = nodeHandle.advertise<std_msgs::Float64MultiArray>("joint_position_controller/command", 1);
    ros::Publisher actuatorsStatePublisher      = nodeHandle.advertise<mildred_core::ActuatorsStateMessage>("actuators_state", 1, true);
    ros::Subscriber jointStatesSubscriber       = nodeHandle.subscribe("joint_states", 1, jointsStatesCallback);

    ros::start();
    ros::Rate rate(100);

    bool sendState = false;

    int testActuator = 8;
    double midpoint = 0;
    double frequency = 0.1;
    double amplitude = 0.75;

    while (ros::ok()) {
        ros::spinOnce();

        if (initialized && ! sendState) {
            ROS_INFO_STREAM("Turning on!");

            midpoint = targetJointPositionMessage.data[testActuator];

            mildred_core::ActuatorsStateMessage message;
            message.state = static_cast<decltype(message.state)>(1);
            actuatorsStatePublisher.publish(message);

            sendState = true;
        }

        if (sendState) {
            double now = ros::Time::now().sec + (ros::Time::now().nsec * 1e-9);
            double value = midpoint + (sin(now / frequency) * amplitude);
            targetJointPositionMessage.data[testActuator] = value;

            targetJointPositionPublisher.publish(targetJointPositionMessage);
        }

        rate.sleep();
    }

    ros::shutdown();

    return 0;
}
