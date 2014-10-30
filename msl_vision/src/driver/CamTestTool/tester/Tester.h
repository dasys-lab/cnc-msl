#ifndef TESTER
#define TESTER

#include "../../frame.h"

//include

//prototype structure: vars and functions that every Tester should have

namespace cameratest
{
    class Tester
    {
            public:
                //vars

                //functions

            protected:
                //vars

                //functions
                virtual void initInternal() = 0;

                virtual void resetInternal() = 0;

                virtual void startTestInternal(camera::Frame frame) = 0;
    };

}



#endif // TESTER
