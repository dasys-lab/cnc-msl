#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "DelayTester.h"

//pure methods implementations

namespace cameratest
{
    //public functions
    DelayTester::DelayTester()
    {
        //Constructor
        initInternal();
    }

    /*DelayTester::~DelayTester()
    {
        //Destructor

    }*/

    void DelayTester::startDelayTest(camera::Frame frame)
    {
        //Constructor
        startTestInternal(frame);
    }




    //protected functions
    void DelayTester::initInternal()
    {
        //protected init
        frames = 0;
        frames_old = 0;
        lThreshold = 70000000;
        usResolx = 640;
        usResoly = 480;

        input = 0;

        memset(&tv1, 0, sizeof(timeval));
        memset(&tv2, 0, sizeof(timeval));

        // memory for debayering
        //int* pbayer;
        //pbayer = (int*)calloc (640 * 480, sizeof(int));

        //camera.printFeatures();
        //camera.DummyTestMethod();

        on=true;
        laston=false;
        gettimeofday(&tv_before, NULL);

    }

    void DelayTester::resetInternal()
    {
        //protected reset
    }

    void DelayTester::startTestInternal(camera::Frame frame)
    {
        //protected startTest

        gettimeofday(&tv1, NULL);
        //Calculate fps
        if (tv1.tv_sec > tv2.tv_sec)
        {
                std::cout << (frames - frames_old) << "fps" << std::endl;

            frames_old = frames;
            tv2 = tv1;
        }

        long long i=0;
        struct timeval tv_after;
        gettimeofday(&tv_after, NULL);
        unsigned char* pbayerc = (unsigned char*) frame.getImagePtr();


        // !!!! have to be the cam resolution
        for (int index = 0; index < (usResolx * usResoly *3); index++)
        {
            i += pbayerc[index];
        }

        //printf("KAKKE %d\n", i);
        //std::cout << std::endl;

        //long timediff = tv_after.tv_usec - tv_before.tv_usec;
        //if(timediff < 0)
        //	timediff += 1000000;

        //150000000 by 1000*1000*3
        //if(i< area*area*30) {
        if (i<lThreshold) //12100480) {
        {
            on = false;
            if (laston != on)
            {

                printf("\n\n\t%ld.%06ld\t%d \n\n", tv_after.tv_sec, tv_after.tv_usec, on);
                std::cout << "" << std::endl;
            }
        }
        else
        {
            on = true;
            if (laston != on)
            {

                printf("\n\n\t%ld.%06ld\t%d \n\n", tv_after.tv_sec, tv_after.tv_usec, on);
                std::cout << "" << std::endl;
            }
        }
        laston = on;

        frames++;

    }


}

