#include "mildred_teleop/MildredTeleop.h"

namespace Mildred {
    MildredTeleop::MildredTeleop():
    nodeHandle() {
        controlPublisher = nodeHandle.advertise<mildred_core::RemoteControlMessage>("control", 1);
    }
}