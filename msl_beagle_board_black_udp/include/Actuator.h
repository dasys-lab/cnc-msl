#pragma once

#include "Includes.h"

#include "BallHandle.h"
#include "IMU.h"
#include "LightBarrier.h"
#include "OpticalFlow.h"
#include "ShovelSelect.h"

struct CV
{
    std::mutex mtx;
    std::condition_variable cv;
    bool notify = false;
};

BlackLib::BlackI2C myI2C(BlackLib::I2C_2, ADR_G);
BlackLib::BlackSPI mySpi(BlackLib::SPI0_0, 8, BlackLib::SpiMode0, 2000000);

const char *IMU_pins[] = {"P8_11", "P8_15", "P8_17", "P8_26"};
const char *OF_pins[] = {"P9_30", "P9_25", "P9_27", "P9_12"};

IMU lsm9ds0(IMU_pins, &myI2C);         /* magnet, accel, temp, gyro Interrupt-Pins */
OpticalFlow adns3080(OF_pins, &mySpi); /* ncs, npd, rst, led */
LightBarrier lightbarrier(BlackLib::AIN0);
ShovelSelect shovel(BlackLib::P9_14); // Delete if using API
// ShovelSelect	shovel(BeaglePWM::P9_14);

timeval time_now;
timeval last_ping;

CV threw[7];

bool ex = false;
