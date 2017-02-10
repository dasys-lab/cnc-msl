#pragma once

#include <vector>

namespace BlackLib
{
  class BlackI2C;
  class BlackSPI;
}

namespace msl_bbb
{

class BallHandler;
class ShovelSelection;
class IMU;
class LightBarrier;
class OpticalFlow;
class Communication;
class Switches;
class BBB
{
  public:
    BBB();
    virtual ~BBB();

    void run();
    static bool running;
    static void sigIntHandler(int sig);

    // for communicating over udp
    Communication *comm;

    /** Actuator & Sensor Worker */
    BallHandler *ballHandler;
    /* Replace with next line for using API */
    ShovelSelection *shovelSelection;
    // ShovelSelection* shovelSelection(BeaglePWM::P9_14);
    OpticalFlow *opticalFlow;
    LightBarrier *lightbarrier;
    IMU *imu;
    Switches * switches;

//    timeval time_now;
//    timeval last_ping;
};
}
