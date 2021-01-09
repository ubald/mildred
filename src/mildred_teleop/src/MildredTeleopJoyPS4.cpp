#include <mildred_core/MildredCommandMessage.h>

#include <mildred_teleop/MildredTeleopJoyPS4.h>

namespace Mildred {

    MildredTeleopJoyPS4::MildredTeleopJoyPS4() :
        MildredTeleopJoy(PS4_BUTTON_COUNT, PS4_AXIS_COUNT) {
    }

    void MildredTeleopJoyPS4::joyCallback(sensor_msgs::Joy joyMsg) {
        MildredTeleopJoy::joyCallback(joyMsg);

        switch (state) {
            case MildredState::Unknown:
                ROS_WARN("Robot state unknown, waiting for robot control");
                break;

            case MildredState::Starting:
                ROS_WARN("Robot state starting, waiting for robot control");
                break;

            case MildredState::Idle:
                if (buttonPressed(PS4_BUTTON_ACTION_CROSS)) {
                    ROS_INFO("Sending sit command");
                    sendCommand(MildredCommand::Sit);
                }
                break;

            case MildredState::Sitting:
                if (buttonPressed(PS4_BUTTON_ACTION_CROSS)) {
                    ROS_INFO("Sending stand command");
                    sendCommand(MildredCommand::Stand);
                } else if (buttonPressed(PS4_BUTTON_ACTION_CIRCLE)) {
                    ROS_INFO("Sending standby command");
                    sendCommand(MildredCommand::Standby);
                } else if (buttonPressed(PS4_BUTTON_ACTION_SQUARE)) {
                    ROS_INFO("Sending ragdoll command");
                    sendCommand(MildredCommand::Ragdoll);
                }
                break;

            case MildredState::Standing:
                if (buttonPressed(PS4_BUTTON_ACTION_CROSS)) {
                    ROS_INFO("Sending walk command");
                    sendCommand(MildredCommand::Walk);
                } else if (buttonPressed(PS4_BUTTON_ACTION_CIRCLE)) {
                    ROS_INFO("Sending sit command");
                    sendCommand(MildredCommand::Sit);
                } else if (buttonPressed(PS4_BUTTON_ACTION_SQUARE)) {
                    ROS_INFO("Sending ragdoll command");
                    sendCommand(MildredCommand::Ragdoll);
                }
                break;

            case MildredState::Walking:
                if (buttonPressed(PS4_BUTTON_ACTION_CIRCLE)) {
                    ROS_INFO("Sending stand command");
                    sendCommand(MildredCommand::Stand);
                } else if (buttonPressed(PS4_BUTTON_ACTION_SQUARE)) {
                    ROS_INFO("Sending ragdoll command");
                    sendCommand(MildredCommand::Ragdoll);
                } else if (axisChanged[PS4_AXIS_STICK_LEFT_X]
                    || axisChanged[PS4_AXIS_STICK_LEFT_Y]
                    || axisChanged[PS4_AXIS_STICK_RIGHT_X]
                    || axisChanged[PS4_AXIS_STICK_RIGHT_Y]
                    || axisChanged[PS4_AXIS_BUTTON_REAR_LEFT_2]
                    || axisChanged[PS4_AXIS_BUTTON_REAR_RIGHT_2]
                    ) {
                    controlMessage.velocity.x = joyMsg.axes[PS4_AXIS_STICK_LEFT_X] * -1;
                    controlMessage.velocity.y = joyMsg.axes[PS4_AXIS_STICK_LEFT_Y];
                    //controlMessage.position.x = joyMsg.axes[PS4_AXIS_BUTTON_CROSS_DOWN] + (joyMsg.axes[PS4_AXIS_BUTTON_CROSS_UP] * -1);
                    //controlMessage.position.y = joyMsg.axes[PS4_AXIS_BUTTON_CROSS_LEFT] + (joyMsg.axes[PS4_AXIS_BUTTON_CROSS_RIGHT] * -1);
                    //controlMessage.position.z = joyMsg.axes[PS4_AXIS_BUTTON_REAR_LEFT_1] + (joyMsg.axes[PS4_AXIS_BUTTON_REAR_RIGHT_1] * -1);
                    //controlMessage.rotation.x = joyMsg.axes[PS4_AXIS_STICK_RIGHT_UPWARDS] * -1;
                    //controlMessage.rotation.y = joyMsg.axes[PS4_AXIS_STICK_RIGHT_LEFTWARDS];
                    //controlMessage.rotation.z = joyMsg.axes[PS4_AXIS_BUTTON_REAR_LEFT_2] + (joyMsg.axes[PS4_AXIS_BUTTON_REAR_RIGHT_2] * -1);

                    controlPublisher.publish(controlMessage);
                }
                break;
        }
    }
}
