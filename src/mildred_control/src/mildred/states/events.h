#pragma once

#include "mildred_control/fsm/Event.h"

namespace Mildred {
    class Enable: public Event {};
    class Ragdoll: public Event {};
    class Sit : public Event {};
    class Stand : public Event {};
    class Walk: public Event {};
}
