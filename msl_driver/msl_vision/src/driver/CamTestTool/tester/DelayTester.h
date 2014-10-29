/*
 * author Dominik Kirchner
 * date: 17.03.2010
 *
 * info: Delay test in use with delay test board (toggle led light)
 *
 * caution: test is camera resolution dependent. lThreshold und usResolx usResoly
 * 			have to be checked
 */

#ifndef CAMERATEST_DELAY
#define CAMERATEST_DELAY

#include <time.h>
#include <sys/time.h>

//#include "Tester.h"
//#include "/home/dki/work/cn/src/Vision5/driver/CamTestTool/tester/Tester.h"
#include "Tester.h"
#include "../../frame.h"

namespace cameratest
{

    class DelayTester : public Tester
    {
        public:
            DelayTester();
            //~DelayTester();

            void startDelayTest(camera::Frame frame);

        protected:

            //vars
            long frames; //uint32_t
            long frames_old;
            long lThreshold;
            unsigned short usResolx, usResoly;

            char cstring[100];
            char input;

            timeval tv1;
            timeval tv2;

            bool on;
            bool laston;
            struct timeval tv_before;


            //functions
            void initInternal();
            void resetInternal();
            void startTestInternal(camera::Frame frame);


    };

}



#endif // CAMERATEST_DELAY
