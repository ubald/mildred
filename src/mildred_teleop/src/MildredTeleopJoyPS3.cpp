#include <mildred_core/MildredCommandMessage.h>

#include <mildred_teleop/MildredTeleopJoyPS3.h>

namespace Mildred {

    MildredTeleopJoyPS3::MildredTeleopJoyPS3() :
        MildredTeleopJoy(PS3_BUTTON_COUNT, PS3_AXIS_COUNT) {
    }

    void MildredTeleopJoyPS3::joyCallback(sensor_msgs::Joy joyMsg) {
        MildredTeleopJoy::joyCallback(joyMsg);

        switch (state) {
            case MildredState::Unknown:
                ROS_WARN("State unknown");
                break;

            case MildredState::Idle:
                ROS_INFO("Idle");
                if (buttonPressed(PS3_BUTTON_ACTION_CROSS)) {
                    ROS_INFO("Sending sit command");
                    sendCommand(MildredCommand::Sit);
                }
                break;

            case MildredState::Sitting:
                ROS_INFO("Sitting");
                if (buttonPressed(PS3_BUTTON_ACTION_CROSS)) {
                    ROS_INFO("Sending stand command");
                    sendCommand(MildredCommand::Stand);
                } else if (buttonPressed(PS3_BUTTON_ACTION_CIRCLE)) {
                    ROS_INFO("Sending Ragdoll command");
                    sendCommand(MildredCommand::Ragdoll);
                }
                break;

            case MildredState::Standing:
                ROS_INFO("Standing");
                if (buttonPressed(PS3_BUTTON_ACTION_CROSS)) {
                    ROS_INFO("Sending walk command");
                    sendCommand(MildredCommand::Walk);
                } else if (buttonPressed(PS3_BUTTON_ACTION_CIRCLE)) {
                    ROS_INFO("Sending sit command");
                    sendCommand(MildredCommand::Sit);
                }

            case MildredState::Walking:
                ROS_INFO("Walking");
                if (buttonPressed(PS3_BUTTON_ACTION_CIRCLE)) {
                    ROS_INFO("Sending stand command");
                    sendCommand(MildredCommand::Stand);
                }

                if (axisChanged[PS3_AXIS_STICK_LEFT_LEFTWARDS]
                    || axisChanged[PS3_AXIS_STICK_LEFT_UPWARDS]
                    || axisChanged[PS3_AXIS_STICK_RIGHT_LEFTWARDS]
                    || axisChanged[PS3_AXIS_STICK_RIGHT_UPWARDS]
                    || axisChanged[PS3_AXIS_BUTTON_REAR_LEFT_1]
                    || axisChanged[PS3_AXIS_BUTTON_REAR_RIGHT_1]
                    || axisChanged[PS3_AXIS_BUTTON_REAR_LEFT_2]
                    || axisChanged[PS3_AXIS_BUTTON_REAR_RIGHT_2]
                    || axisChanged[PS3_AXIS_BUTTON_CROSS_DOWN]
                    || axisChanged[PS3_AXIS_BUTTON_CROSS_UP]
                    || axisChanged[PS3_AXIS_BUTTON_CROSS_LEFT]
                    || axisChanged[PS3_AXIS_BUTTON_CROSS_RIGHT]
                    ) {
                    controlMessage.velocity.x = joyMsg.axes[PS3_AXIS_STICK_LEFT_LEFTWARDS] * -1;
                    controlMessage.velocity.y = joyMsg.axes[PS3_AXIS_STICK_LEFT_UPWARDS];
                    controlMessage.position.x = joyMsg.axes[PS3_AXIS_BUTTON_CROSS_DOWN] + (joyMsg.axes[PS3_AXIS_BUTTON_CROSS_UP] * -1);
                    controlMessage.position.y = joyMsg.axes[PS3_AXIS_BUTTON_CROSS_LEFT] + (joyMsg.axes[PS3_AXIS_BUTTON_CROSS_RIGHT] * -1);
                    controlMessage.position.z = joyMsg.axes[PS3_AXIS_BUTTON_REAR_LEFT_1] + (joyMsg.axes[PS3_AXIS_BUTTON_REAR_RIGHT_1] * -1);
                    controlMessage.rotation.x = joyMsg.axes[PS3_AXIS_STICK_RIGHT_UPWARDS] * -1;
                    controlMessage.rotation.y = joyMsg.axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS];
                    controlMessage.rotation.z = joyMsg.axes[PS3_AXIS_BUTTON_REAR_LEFT_2] + (joyMsg.axes[PS3_AXIS_BUTTON_REAR_RIGHT_2] * -1);

                    controlPublisher.publish(controlMessage);
                }
                break;
        }
    }
}
