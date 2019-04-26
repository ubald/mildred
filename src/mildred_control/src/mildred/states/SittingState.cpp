#include "SittingState.h"

namespace Mildred {
    SittingState::SittingState(MildredControl * control) :
        State(MildredState::Sitting, "sitting"),
        control(control){}
}