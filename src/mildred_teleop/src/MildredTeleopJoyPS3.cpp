#include "mildred_teleop/MildredTeleopJoyPS3.h"


namespace Mildred {

    MildredTeleopJoyPS3::MildredTeleopJoyPS3() :
        MildredTeleopJoy(PS3_BUTTON_COUNT, PS3_AXIS_COUNT) {
    }

    void MildredTeleopJoyPS3::joyCallback(sensor_msgs::Joy joyMsg) {
        MildredTeleopJoy::joyCallback(joyMsg);

        //Robot Control
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

        //sensor_msgs::JoyFeedback rumbleMsg;
        //rumbleMsg.type = ledMsg.TYPE_RUMBLE;
        //rumbleMsg.id = 0;
        //rumbleMsg.intensity = fabs(joyMsg.axes[PS3_AXIS_BUTTON_REAR_LEFT_2]);
        //sensor_msgs::JoyFeedbackArray jfMsg;
        //jfMsg.array.resize(1);
        //jfMsg.array[0] = rumbleMsg;
        //joyFeedbackPublisher.publish( jfMsg );

        /*if ( joyMsg.buttons[PS3_BUTTON_START] == 1 && !buttonState[PS3_BUTTON_START] )
        {
            sensor_msgs::JoyFeedback ledMsg;
            ledMsg.type = ledMsg.TYPE_LED;
            ledMsg.id = PS3_LED_1;
            ledMsg.intensity = 1.0f;

            sensor_msgs::JoyFeedback rumbledMsg;
            rumbledMsg.type = ledMsg.TYPE_RUMBLE;
            rumbledMsg.id = 1;
            rumbledMsg.intensity = 1.0f;

            sensor_msgs::JoyFeedbackArray jfMsg;
            jfMsg.array.resize(2);
            jfMsg.array[0] = ledMsg;
            jfMsg.array[1] = rumbledMsg;

            joyFeedbackPublisher.publish( jfMsg );

            buttonState[PS3_BUTTON_START] = true;
        }
        else if ( joyMsg.buttons[PS3_BUTTON_START] == 0 && buttonState[PS3_BUTTON_START] )
        {
            sensor_msgs::JoyFeedback ledMsg;
            ledMsg.type = ledMsg.TYPE_LED;
            ledMsg.id = PS3_LED_1;
            ledMsg.intensity = 0.0f;

            sensor_msgs::JoyFeedback rumbledMsg;
            rumbledMsg.type = ledMsg.TYPE_RUMBLE;
            rumbledMsg.id = 1;
            rumbledMsg.intensity = 0.0f;

            sensor_msgs::JoyFeedbackArray jfMsg;
            jfMsg.array.resize(2);
            jfMsg.array[0] = ledMsg;
            jfMsg.array[1] = rumbledMsg;

            joyFeedbackPublisher.publish( jfMsg );

            buttonState[PS3_BUTTON_START] = false;
        }*/
    }
}
