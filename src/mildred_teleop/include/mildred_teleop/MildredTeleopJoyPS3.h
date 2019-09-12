#pragma once

#include <ros/ros.h>

#include "mildred_teleop/MildredTeleopJoy.h"

// BUTTONS
// buttons are either 0 or 1
#define PS3_BUTTON_COUNT                 17

#define PS3_BUTTON_SELECT                0
#define PS3_BUTTON_STICK_LEFT            1
#define PS3_BUTTON_STICK_RIGHT           2
#define PS3_BUTTON_START                 3
#define PS3_BUTTON_CROSS_UP              4
#define PS3_BUTTON_CROSS_RIGHT           5
#define PS3_BUTTON_CROSS_DOWN            6
#define PS3_BUTTON_CROSS_LEFT            7
#define PS3_BUTTON_REAR_LEFT_2           8
#define PS3_BUTTON_REAR_RIGHT_2          9
#define PS3_BUTTON_REAR_LEFT_1           10
#define PS3_BUTTON_REAR_RIGHT_1          11
#define PS3_BUTTON_ACTION_TRIANGLE       12
#define PS3_BUTTON_ACTION_CIRCLE         13
#define PS3_BUTTON_ACTION_CROSS          14
#define PS3_BUTTON_ACTION_SQUARE         15
#define PS3_BUTTON_PAIRING               16

// AXES
// stick axes go from 0 to +/-1
// button axes go from 0 to -1
#define PS3_AXIS_COUNT                   20

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19

// LEDS
#define PS3_LED_COUNT                    4

#define PS3_LED_1                        0
#define PS3_LED_2                        1
#define PS3_LED_3                        2
#define PS3_LED_4                        3

namespace Mildred {

    class MildredTeleopJoyPS3: public MildredTeleopJoy {
    public:
        MildredTeleopJoyPS3();
        ~MildredTeleopJoyPS3() = default;
    protected:
        void joyCallback(sensor_msgs::Joy joyMsg) override;
    };
}