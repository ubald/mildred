#pragma once

#include <mildred_control/fsm/Machine.h>
#include <mildred_core/MildredControlMessage.h>

namespace Mildred {
    class ControlState;
    class ControlMachine: public Machine<ControlState,ControlMachine> {
      public:
        void handleControl(const mildred_core::MildredControlMessage::ConstPtr &controlMessage);
    };
}
