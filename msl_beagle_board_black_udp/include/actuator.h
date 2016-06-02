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


<<<<<<< HEAD
BallHandle		BH_right(BeaglePWM::P8_19, BH_right_pins);		/* pwm, dir, reset, ff1, ff2 */
BallHandle		BH_left(BeaglePWM::P8_13, BH_left_pins);			/* pwm, dir, reset, ff1, ff2 */
IMU				lsm9ds0(IMU_pins, &myI2C);		/* magnet, accel, temp, gyro Interrupt-Pins */
OpticalFlow		adns3080(OF_pins, &mySpi);	/* ncs, npd, rst, led */
LightBarrier	lightbarrier(AIN0);
ShovelSelect	shovel(BeaglePWM::P9_14);
=======
BallHandle		BH_right(P8_19, GPIO_69, GPIO_68, GPIO_46, GPIO_65);		/* pwm, dir, reset, ff1, ff2 */
BallHandle		BH_left(P8_13, GPIO_66, GPIO_67, GPIO_44, GPIO_26);			/* pwm, dir, reset, ff1, ff2 */
IMU				lsm9ds0(GPIO_45, GPIO_47, GPIO_27, GPIO_61, &myI2C);		/* magnet, accel, temp, gyro Interrupt-Pins */
LightBarrier	lightbarrier(AIN0);
OpticalFlow		adns3080(GPIO_112, GPIO_117, GPIO_115, GPIO_60, &mySpi);	/* ncs, npd, rst, led */
ShovelSelect	shovel(P9_14);
>>>>>>> master

timeval			time_now;
timeval			last_ping;

CV				threw[7];

<<<<<<< HEAD
=======

>>>>>>> master
bool			ex = false;


#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_ACTUATOR_H_ */
