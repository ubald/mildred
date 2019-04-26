#include <utility>

#pragma once

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <mildred_core/mildred.h>
#include <mildred_core/MildredControlMessage.h>

#include "Event.h"

namespace Mildred {
    class Machine;

    class State {
        friend class Machine;

      public:
        ~State() = default;

        virtual MildredState id() const { return id_; };
        virtual std::string name() const { return name_; };

        virtual bool onEnter(const Event &event) { return true; }

        virtual bool onExit(const Event &event) { return true; };

        virtual void tick(double now, double delta) {};

        virtual void handleControl(const mildred_core::MildredControlMessage::ConstPtr &controlMessage) {};

      protected:
        State(MildredState id, std::string name) : id_(id), name_(std::move(name)) {};

        MildredState id_;
        std::string name_;

        Machine *machine{nullptr};

      private:
    };
}
