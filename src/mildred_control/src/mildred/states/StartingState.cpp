#include "StartingState.h"

namespace Mildred {
    StartingState::StartingState(MildredControl *control) : ControlState(MildredState::Starting, "starting", control) {}
}