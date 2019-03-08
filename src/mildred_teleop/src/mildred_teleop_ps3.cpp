#include "mildred_teleop/mildred_teleop_ps3.h"

int main(int argc, char **argv) {
    //Initialize values
    for (int i = 0; i < PS3_BUTTON_COUNT; ++i) {
        lastButtonValue[i] = false;
    }
    for (int i = 0; i < PS3_AXIS_COUNT; ++i) {
        lastAxisValue[i] = 0.00f;
    }
    for (int i = 0; i < PS3_AXIS_COUNT; ++i) {
        axisChanged[i] = false;
    }


    ros::init(argc, argv, "mildred_teleop_ps3");
    ros::NodeHandle n;

    joySubscriber = n.subscribe("/joy", 1, joyCallback);

    joyFeedbackPublisher = n.advertise<sensor_msgs::JoyFeedbackArray>("/joy/set_feedback", 1);
    controlPublisher = n.advertise<mildred_core::RemoteControlMessage>("mildred/control", 1);
    //TODO: Gait change publisher

    ros::spin();
    return 0;
}

void joyCallback(sensor_msgs::Joy joyMsg) {
    //Generic Button events
    for (int i = 0; i < PS3_BUTTON_COUNT; ++i) {
        if (joyMsg.buttons[i] != lastButtonValue[i]) {
            if (joyMsg.buttons[i]) {
                //Button pressed
                ROS_DEBUG("Button pressed");
            } else {
                //Button released
                ROS_DEBUG("Button released");
            }

            lastButtonValue[i] = joyMsg.buttons[i];
        }
    }

    //Generic Axis Events
    for (int i = 0; i < PS3_AXIS_COUNT; ++i) {
        if (joyMsg.axes[i] != lastAxisValue[i]) {
            //Axis Changed
            axisChanged[i] = true;
            lastAxisValue[i] = joyMsg.axes[i];
        } else {
            axisChanged[i] = false;
        }
    }

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
        control.velocity.x = joyMsg.axes[PS3_AXIS_STICK_LEFT_UPWARDS];
        control.velocity.y = joyMsg.axes[PS3_AXIS_STICK_LEFT_LEFTWARDS] * -1;
        control.position.x = joyMsg.axes[PS3_AXIS_BUTTON_CROSS_DOWN] +
                             (joyMsg.axes[PS3_AXIS_BUTTON_CROSS_UP] * -1);
        control.position.y = joyMsg.axes[PS3_AXIS_BUTTON_CROSS_LEFT] +
                             (joyMsg.axes[PS3_AXIS_BUTTON_CROSS_RIGHT] * -1);
        control.position.z = joyMsg.axes[PS3_AXIS_BUTTON_REAR_LEFT_1] +
                             (joyMsg.axes[PS3_AXIS_BUTTON_REAR_RIGHT_1] * -1);
        control.rotation.x = joyMsg.axes[PS3_AXIS_STICK_RIGHT_UPWARDS] * -1;
        control.rotation.y = joyMsg.axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS];
        control.rotation.z = joyMsg.axes[PS3_AXIS_BUTTON_REAR_LEFT_2] +
                             (joyMsg.axes[PS3_AXIS_BUTTON_REAR_RIGHT_2] * -1);
        controlPublisher.publish(control);
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
