#pragma once

#include <ros/ros.h>

#include "mildred_core/RemoteControlMessage.h"
#include "mildred_teleop/MildredTeleopJoy.h"

// BUTTONS
// buttons are either 0 or 1
#define PS4_BUTTON_COUNT                 13

#define PS4_BUTTON_ACTION_CROSS          0
#define PS4_BUTTON_ACTION_CIRCLE         1
#define PS4_BUTTON_ACTION_TRIANGLE       2
#define PS4_BUTTON_ACTION_SQUARE         3
#define PS4_BUTTON_REAR_LEFT_1           4
#define PS4_BUTTON_REAR_RIGHT_1          5
#define PS4_BUTTON_REAR_LEFT_2           6
#define PS4_BUTTON_REAR_RIGHT_2          7
#define PS4_BUTTON_SHARE                 8
#define PS4_BUTTON_OPTIONS               9
#define PS4_BUTTON_PS                    10
#define PS4_BUTTON_STICK_LEFT            11
#define PS4_BUTTON_STICK_RIGHT           12


// AXES
// stick axes go from 0 to +/-1
// button axes go from 0 to -1
#define PS4_AXIS_COUNT                   8

#define PS4_AXIS_STICK_LEFT_X            0
#define PS4_AXIS_STICK_LEFT_Y            1
#define PS4_AXIS_BUTTON_REAR_LEFT_2      2
#define PS4_AXIS_STICK_RIGHT_X           3
#define PS4_AXIS_STICK_RIGHT_Y           4
#define PS4_AXIS_BUTTON_REAR_RIGHT_2     5
#define PS4_AXIS_BUTTON_CROSS_X          6
#define PS4_AXIS_BUTTON_CROSS_Y          7

namespace Mildred {

    class MildredTeleopJoyPS4 : public MildredTeleopJoy {
      public:
        MildredTeleopJoyPS4();
        ~MildredTeleopJoyPS4() = default;
      protected:
        void joyCallback(sensor_msgs::Joy joyMsg) override;
    };
}