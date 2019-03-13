#ifndef TRIPODGAIT_H_
#define TRIPODGAIT_H_

#include "Gait.h"

namespace UWalker
{
    
    class ContinuousGait: public UWalker::Gait
    {
        public:
            ContinuousGait( UWalker::EGaitSequence sequence );
            virtual ~ContinuousGait();

            void setup();

        protected:
            KDL::Vector calculate( UWalker::GaitConfig &legGait );
            double posx, posy, posz;
    };

}
#endif
