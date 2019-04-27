#pragma once

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

#include "Event.h"

namespace Mildred {

    template<class M>
    class State {
      public:
        ~State() = default;

        M *machine{nullptr};

        virtual std::string name() const { return name_; };

        virtual bool onEnter(const Event &event) { return true; }

        virtual bool onExit(const Event &event) { return true; };

        virtual void tick(double now, double delta) {};

      protected:
        explicit State(std::string name) : name_(std::move(name)) {};

        std::string name_;


      private:
    };
}
