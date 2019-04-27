#pragma once

#include "Gait.h"

namespace Mildred {

    class ContinuousGait : public Gait {
    public:
        explicit ContinuousGait(GaitSequence sequence);
        ~ContinuousGait() = default;

        void setup() override;

    protected:
        KDL::Vector calculate(GaitConfig &legGait) override;
        double posx;
        double posy;
        double posz;
    };

}