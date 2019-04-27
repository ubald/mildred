#pragma once

#include <any>
#include <functional>
#include <iostream>
#include <memory>
#include <typeindex>
#include <typeinfo>
#include <type_traits>
#include <vector>

#include "Event.h"
#include "State.h"
#include "Transition.h"

namespace Mildred {
    template<class M, class S>
    class Machine {
      public:
        Machine() = default;
        ~Machine() = default;

        std::shared_ptr<S> state() const {
            return currentState;
        }

        M* addState(const std::shared_ptr<S> &state, bool initial = false) {
            state->machine = static_cast<M*>(this);
            states.push_back(state);

            if (initial) {
                currentState = state;
                currentState->onEnter(Event());
                notifyStateChange();
            }

            return static_cast<M*>(this);
        }

        template<typename T, typename... Args>
        std::shared_ptr<T> addState(Args &&... args) {
            auto state = std::make_shared<T>(std::forward<Args>(args)...);
            std::static_pointer_cast<T>(addState(state));
            return state;
        }

        template<
            class E,
            class FS,
            class TS,
            typename = std::enable_if<std::is_base_of<Event, E>::value>,
            typename = std::enable_if<std::is_base_of<std::shared_ptr<S>, FS>::value>,
            typename = std::enable_if<std::is_base_of<std::shared_ptr<S>, TS>::value>>
        M* addTransition(std::shared_ptr<FS> fromState, std::shared_ptr<TS> toState) {
            std::cout << "Adding transition from state " << fromState->name() << " to state " << toState->name() << " for event " << typeid(E).name() << " (" << typeid(E).hash_code() << ")" << std::endl;
            transitions[fromState].emplace(typeid(E), std::static_pointer_cast<BaseTransition<S, E>>(std::make_shared<Transition<S, E, FS, TS>>(fromState, toState)));
            return static_cast<M*>(this);
        }

        template<class E, typename = std::enable_if<std::is_base_of<Event, E>::value>>
        void handleEvent(const E &event) {
            if (currentState == nullptr) {
                std::cerr << "No current state" << std::endl;
                return;
            }

            auto possibleTransitions = transitions.find(currentState);
            if (possibleTransitions == transitions.end()) {
                std::cerr << "No possible transitions from state " << currentState->name() << std::endl;
                return;
            }

            auto transition = possibleTransitions->second.find(typeid(E));
            if (transition == possibleTransitions->second.end()) {
                std::cerr << "No transition from state " << currentState->name() << " for event " << typeid(E).name() << " (" << typeid(E).hash_code() << ")" << std::endl;
                return;
            }

            auto newState = std::any_cast<std::shared_ptr<BaseTransition<S, E>>>(transition->second)->transit(event);
            if (newState && newState != currentState) {
                currentState = newState;
                notifyStateChange();
            }
        }

        void notifyStateChange() {
            if (onStateChange != nullptr) {
                onStateChange(currentState);
            }
        }

        void tick(double now, double delta) {
            if (currentState == nullptr) {
                return;
            }

            currentState->tick(now, delta);
        }

        std::function<void(std::shared_ptr<S> state)> onStateChange{nullptr};

      protected:
        std::vector<std::shared_ptr<S>>                                                       states;
        std::shared_ptr<S>                                                                    currentState;
        std::unordered_map<std::shared_ptr<S>, std::unordered_map<std::type_index, std::any>> transitions;
    };
}