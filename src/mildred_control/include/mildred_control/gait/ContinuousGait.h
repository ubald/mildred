#pragma once

#include "Gait.h"

namespace Mildred {

    class ContinuousGait : public Gait {
    public:
        explicit ContinuousGait(GaitSequence sequence);
        ~ContinuousGait() = default;

        void setup() override;

    protected:
        tf2::Vector3 calculate(GaitConfig &legGait) override;
        double posx;
        double posy;
        double posz;
    };

}