#include "mildred_control/gait/Gait.h"

namespace Mildred {
    Gait::Gait(GaitSequence sequence) {
        switch (sequence) {
            case TRIPOD:
                flightTime = 0.50; // Percentage of the time the leg is in the air
                timing.push_back(0.000);
                timing.push_back(0.500);
                timing.push_back(0.000);
                timing.push_back(0.500);
                timing.push_back(0.000);
                timing.push_back(0.500);
                break;

            case WAVE:
                flightTime = 0.1667; // Percentage of the time the leg is in the air
                timing.push_back(0.333);
                timing.push_back(0.1667);
                timing.push_back(0.000);
                timing.push_back(0.833);
                timing.push_back(0.6667);
                timing.push_back(0.500);
                break;

            case RIPPLE:
                flightTime = 0.333; // Percentage of the time the leg is in the air
                timing.push_back(0.6667); //0:5
                timing.push_back(0.333);  //1:3
                timing.push_back(0.000);  //2:1
                timing.push_back(0.1667);  //3:4
                timing.push_back(0.833);  //4:6
                timing.push_back(0.500); //5:2
                break;
        }

        //stepDuration = cycleDuration / cycle.steps;
    }

    void Gait::setup() {
        lastTime      = ros::Time::now().sec + (ros::Time::now().nsec * 1e-9);
        cycleTime     = 0.00;
        direction     = 0.00;
        cyclePosition = 0.00;

        //stepPosition = 0.00;
    }

    void Gait::prepare(double in_speed, double in_direction) {
        speed     = in_speed;
        direction = in_direction;

        double now = ros::Time::now().sec + (ros::Time::now().nsec * 1e-9);
        double dt  = now - lastTime;
        lastTime = now;

        //Find where we are in our cycle depending on speed
        cycleTime += dt * cycleTimeMultiplier * speed;
        if (cycleTime > cycleDuration) {
            cycleTime -= cycleDuration;
        } else if (cycleTime < 0) {
            cycleTime += cycleDuration;
        }
    }

    KDL::Vector Gait::walk(GaitConfig &legGait) {
        cyclePosition = (cycleTime / cycleDuration) + timing[legGait.index];
        if (cyclePosition > 1.00) {
            cyclePosition = cyclePosition - 1;
        }

        //Find position in current step
        //double cycleStep = cycleTime / stepDuration;
        //step = floor( cycleStep );
        //stepPosition = cycleStep - step;

        return calculate(legGait);
    }
}
