#pragma once

#include "Gait.h"

namespace Mildred {

    class ContinuousGait : public Mildred::Gait {
    public:
        explicit ContinuousGait(Mildred::EGaitSequence sequence);
        ~ContinuousGait() = default;

        void setup() override;

    protected:
        KDL::Vector calculate(Mildred::GaitConfig &legGait) override;
        double posx;
        double posy;
        double posz;
    };

}