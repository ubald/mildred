#include "ContinuousGait.h"

namespace UWalker
{
    ContinuousGait::ContinuousGait( UWalker::EGaitSequence sequence ) :
            Gait::Gait( sequence )
    {
        cycleTimeMultiplier = 2.500;
        stepLength = 0.100; // Length of a step
        stepHeight = 0.075; // Height of a step
        bodyHeight = 0.060; // Body height
        radius = 0.125;
    }
    
    ContinuousGait::~ContinuousGait()
    {
        // TODO Auto-generated destructor stub
    }

    void ContinuousGait::setup()
    {
        Gait::setup();
    }

    KDL::Vector ContinuousGait::calculate( UWalker::GaitConfig &legGait )
    {
        double stanceDuration = 1 - flightTime;

        if ( cyclePosition <= stanceDuration )
        {
            //Stance
            posx = ( stepLength * ( cyclePosition / stanceDuration ) ) - ( stepLength / 2 );
            posz = -bodyHeight;
        }
        else
        {
            //Flight
            double pos = ( cyclePosition - stanceDuration ) / ( 1 - stanceDuration );
            posx = cos( pos * M_PI ) * ( stepLength / 2 );
            posz = ( sin( pos * M_PI ) * stepHeight ) - bodyHeight;
        }

        // TODO Changes in directions should be smooth (PID Loop?) Currently it jumps on the other side of the gait loop

        // NOTE -M_PI_2 because, well, I don't know, the direction was off by 90 degrees

        posy = posx * cos( direction + legGait.alpha - M_PI_2 );
        posx = posx * sin( direction + legGait.alpha - M_PI_2 ) + radius;

        return KDL::Vector( posx, posy, posz );
    }

}
