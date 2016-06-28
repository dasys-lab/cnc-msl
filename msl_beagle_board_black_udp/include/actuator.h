/*
 * actuator.h
 *
 *  Created on: Apr 7, 2015
 *      Author: Lukas Will
 */

#ifndef CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_ACTUATOR_H_
#define CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_ACTUATOR_H_

#include "includes.h"

#include "ballhandle.h"
#include "imu.h"
#include "lightbarrier.h"
#include "opticalflow.h"
#include "shovelselect.h"

using namespace BlackLib;


struct CV {
	std::mutex					mtx;
	std::condition_variable		cv;
	bool						notify = false;
};

BlackI2C myI2C(I2C_2, ADR_G);
BlackSPI mySpi(SPI0_0, 8, SpiMode0, 2000000);

const char *IMU_pins[] = { "P8_11", "P8_15", "P8_17", "P8_26" };
const char *OF_pins[] = { "P9_30", "P9_25", "P9_27", "P9_12" };

IMU				lsm9ds0(IMU_pins, &myI2C);		/* magnet, accel, temp, gyro Interrupt-Pins */
OpticalFlow		adns3080(OF_pins, &mySpi);	/* ncs, npd, rst, led */
LightBarrier	lightbarrier(AIN0);
ShovelSelect	shovel(P9_14);	// Delete if using API
//ShovelSelect	shovel(BeaglePWM::P9_14);

timeval			time_now;
timeval			last_ping;

CV				threw[7];

bool			ex = false;


#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_ACTUATOR_H_ */
