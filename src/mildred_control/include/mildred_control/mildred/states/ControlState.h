#pragma once

#include <mildred_core/mildred.h>
#include <mildred_core/MildredControlMessage.h>
#include <mildred_control/fsm/State.h>
#include <mildred_control/MildredControl.h>
#include <mildred_control/mildred/states/ControlMachine.h>

namespace Mildred {
    class ControlState: public State<ControlMachine> {
      public:
        ControlState(MildredState id, std::string name, MildredControl * control);
        virtual MildredState id() const { return id_; };
        virtual void handleControl(const mildred_core::MildredControlMessage::ConstPtr &controlMessage);

      protected:
        MildredState id_;
        MildredControl *control_{nullptr};
    };
}
