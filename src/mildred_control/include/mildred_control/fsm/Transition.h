#pragma once

#include <memory>
#include <type_traits>

#include "Event.h"

namespace Mildred {
    template<class E, typename = std::enable_if<std::is_base_of<Event, E>::value>>
    class BaseTransition {
        public:
            virtual bool transit(const E &event) = 0;
    };

    template<
        class E,
        class FS,
        class TS,
        typename = std::enable_if<std::is_base_of<Event, E>::value>,
        typename = std::enable_if<std::is_base_of<std::shared_ptr<State>, FS>::value>,
    typename = std::enable_if<std::is_base_of<std::shared_ptr<State>, TS>::value>>
    class Transition : public BaseTransition<E> {
        public:
            Transition(std::shared_ptr<FS> fromState, std::shared_ptr<TS> toState) :
                fromState(fromState), toState(toState) {};
            ~Transition() = default;

            bool transit(const E &event) override {
                std::cout << "templated" << std::endl;
                return fromState->onExit(event) && toState->onEnter(event);
            }

        private:
            std::shared_ptr<FS> fromState{nullptr};
            std::shared_ptr<TS> toState{nullptr};
    };
}