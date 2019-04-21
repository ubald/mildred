#pragma once

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

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

        protected:
            State() = default;
            Machine * machine{nullptr};

        private:
    };
}
