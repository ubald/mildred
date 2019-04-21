#include "mildred_control/fsm/Machine.h"

namespace Mildred {
    Machine::Machine() {

    }

    void Machine::addState(std::shared_ptr<State> state, bool initial) {
        state->machine = this;
        states.push_back(state);

        if (initial) {
            currentState = state;
        }
    }
}