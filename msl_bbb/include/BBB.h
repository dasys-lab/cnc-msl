#pragma once

#include <BallHandler.h>
#include <ShovelSelection.h>
#include "Includes.h"

#include "IMU.h"
#include "LightBarrier.h"
#include "OpticalFlow.h"
#include <vector>

namespace msl_bbb
{

struct CV
{
    std::mutex mtx;
    std::condition_variable cv;
    bool notify = false;
};

class BBB
{
  public:
    BBB();
    virtual ~BBB();

    void run();
    static bool running;
    static void sigIntHandler(int sig);

    // for communicating over udp
    Communication* comm;

    /** Actuator & Sensor Instances */

    BallHandler* ballHandler;

    /* Replace with next line for using API */
    ShovelSelection* shovelSelection;
    // ShovelSelection* shovelSelection(BeaglePWM::P9_14);

    OpticalFlow* opticalFlow; //(OF_pins, &mySpi); /* ncs, npd, rst, led */



    LightBarrier lightbarrier; //(BlackLib::AIN0);
    IMU lsm9ds0; //(IMU_pins, &myI2C);         /* magnet, accel, temp, gyro Interrupt-Pins */

    // onboard communication
    BlackLib::BlackI2C myI2C; //(BlackLib::I2C_2, ADR_G);
    BlackLib::BlackSPI mySpi; //(BlackLib::SPI0_0, 8, BlackLib::SpiMode0, 2000000);

    std::vector<char const*>IMU_pins;
    std::vector<char const*>OF_pins;

    timeval time_now;
    timeval last_ping;

    CV threw[7];


};
}
