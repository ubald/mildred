#pragma once

#include <ros/ros.h>
#include <kdl/frames.hpp>

namespace Mildred {
    enum EGaitSequence {
        TRIPOD,
        WAVE,
        RIPPLE
    };

    struct GaitConfig {
        unsigned int index;
        double       alpha;
    };

    class Gait {
    public:
        virtual void setup() = 0;
        virtual void prepare(double speed, double direction);
        virtual KDL::Vector walk(Mildred::GaitConfig &legGait);

    protected:
        explicit Gait(Mildred::EGaitSequence sequence);
        //virtual ~Gait();
        virtual KDL::Vector calculate(Mildred::GaitConfig &legGait) = 0;

        static constexpr const double cycleDuration = 1.00; // Duration of a cycle, PI could be used also

        std::vector<double> timing;

        //Config values
        double cycleTimeMultiplier;  // Global time multiplier
        double flightTime;           // Percentage of the time the leg is on the floor
        double stepLength;           // Length of a step
        double stepHeight;           // Height of a step
        double bodyHeight;           // Body height
        double radius;               // Gait radius

        //Inputs
        double speed;
        double direction;

        //Run-time values
        double cyclePosition;        // Position in the cycle
        double lastTime;

    private:
        double cycleTime;





        //unsigned int step;           // Current step
        //double stepDuration;         // Duration of a step (fraction of cycleDuration)
        //double stepPosition;         // Current position inside step (between 0.00 and 1.00)

    };

}