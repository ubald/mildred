#include "mildred_control/gait/ContinuousGait.h"

namespace Mildred {
    ContinuousGait::ContinuousGait(GaitSequence sequence) :
        Gait::Gait(sequence) {
        cycleTimeMultiplier = 2.500;
        stepLength          = 0.150;
        stepHeight          = 0.050;
        bodyHeight          = 0.100;
        radius              = 0.200;
    }

    void ContinuousGait::setup() {
        Gait::setup();
    }

    tf2::Vector3 ContinuousGait::calculate(GaitConfig &legGait) {
        double stanceDuration = 1 - flightTime;

        if (cyclePosition <= stanceDuration) {
            // Stance
            posx = (stepLength * (cyclePosition / stanceDuration)) - (stepLength / 2);
            posz = -bodyHeight;
        } else {
            // Flight
            double pos = (cyclePosition - stanceDuration) / (1 - stanceDuration);
            posx = cos(pos * M_PI) * (stepLength / 2);
            posz = (sin(pos * M_PI) * stepHeight) - bodyHeight;
        }

        // TODO Changes in directions should be smooth (PID Loop?) Currently it jumps on the other side of the gait loop

        // NOTE -M_PI_2 because, well, I don't know, the direction was off by 90 degrees

        posy = posx * cos(direction + legGait.alpha - M_PI_2);
        posx = posx * sin(direction + legGait.alpha - M_PI_2) + radius;

        return tf2::Vector3(posx, posy, posz);
    }

}
