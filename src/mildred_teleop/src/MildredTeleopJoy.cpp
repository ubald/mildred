#include "mildred_teleop/MildredTeleopJoy.h"

namespace Mildred {
    MildredTeleopJoy::MildredTeleopJoy(std::size_t buttonCount, std::size_t axesCount) :
        MildredTeleop(),
        lastButtonValue(buttonCount, false),
        lastAxisValue(axesCount, 0.00f),
        axisChanged(axesCount, false) {
        joySubscriber = nodeHandle.subscribe("/joy", 1, &MildredTeleopJoy::joyCallback, this);
        joyFeedbackPublisher = nodeHandle.advertise<sensor_msgs::JoyFeedbackArray>("/joy/set_feedback", 1);
    }

    void MildredTeleopJoy::joyCallback(sensor_msgs::Joy joyMsg) {
        // Generic Button events
        if (joyMsg.buttons.size() > lastButtonValue.size()) {
            ROS_WARN("Received more buttons than supported.");
        } else if (joyMsg.buttons.size() < lastButtonValue.size()) {
            ROS_WARN("Received less buttons than expected.");
        }

        auto buttonCount = std::min(joyMsg.buttons.size(), lastButtonValue.size());

        for (int i = 0; i < buttonCount; ++i) {
            if (joyMsg.buttons[i] != lastButtonValue[i]) {
                if (joyMsg.buttons[i]) {
                    ROS_DEBUG("Button pressed");
                } else {
                    ROS_DEBUG("Button released");
                }

                lastButtonValue[i] = joyMsg.buttons[i] != 0;
            }
        }

        // Generic Axis Events
        if (joyMsg.axes.size() > lastAxisValue.size()) {
            ROS_WARN("Received more axes than supported.");
        } else if (joyMsg.axes.size() < lastAxisValue.size()) {
            ROS_WARN("Received less axes than expected.");
        }

        auto axesCount = std::min(joyMsg.axes.size(), lastAxisValue.size());

        for (int i = 0; i < axesCount; ++i) {
            if (joyMsg.axes[i] != lastAxisValue[i]) {
                axisChanged[i]   = true;
                lastAxisValue[i] = joyMsg.axes[i];
            } else {
                axisChanged[i] = false;
            }
        }
    }
}