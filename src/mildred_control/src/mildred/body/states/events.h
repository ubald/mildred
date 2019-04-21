#pragma once

#include "mildred_control/fsm/Event.h"

namespace Mildred {
    class Stand : public Event {};

    class Sit : public Event {};

    class Ragdoll: public Event {};
}
