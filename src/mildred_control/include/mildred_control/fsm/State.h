#pragma once

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

#include <mildred_core/RemoteControlMessage.h>

#include "Event.h"

namespace Mildred {
    class Machine;

    class State {
        friend class Machine;

        public:
            ~State() = default;
            virtual std::string name() const = 0;
            virtual bool onEnter(const Event &event) { return true; }
            virtual bool onExit(const Event &event) { return true; };
            virtual void tick(double now, double delta) {};
            virtual void handleControl(const mildred_core::RemoteControlMessage::ConstPtr &controlMessage) {};

        protected:
            State() = default;
            Machine * machine{nullptr};

        private:
    };
}
