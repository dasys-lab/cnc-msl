/*
 * actuator.h
 *
 *  Created on: Apr 7, 2015
 *      Author: Lukas Will
 */

#ifndef CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_ACTUATOR_H_
#define CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_ACTUATOR_H_


#include "config.h"
#include "ballhandle.h"
#include "opticalflow.h"
#include "imu.h"

#include <mutex>
#include <condition_variable>

/*
Axis.msg

int16_t x;
int16_t y;
int16_t z;


IMUInfo.msg

Axis	accel;
Axis	gyro;
Axis	magnet;
int16_t temperature;

 */


using namespace BlackLib;


struct Shovel {
	bool		enabled;
	uint16_t	value;
	timeval		last_ping;
};

struct CV {
	std::mutex					mtx;
	std::condition_variable		cv;
	bool						notify = false;
};


BlackGPIO i_magnet(GPIO_45, input, FastMode);		// P8 11
BlackGPIO i_accel(GPIO_47, input, FastMode);		// P8 15
BlackGPIO i_temp(GPIO_27, input, FastMode);			// P8 17
BlackGPIO i_gyro(GPIO_61, input, FastMode);			// P8 26

BlackGPIO LED_Vision(GPIO_49, output, FastMode);	// P9 23
BlackGPIO LED_Bundle(GPIO_20, output, FastMode);	// P9 41
BlackGPIO LED_Power(GPIO_7, output, FastMode);		// P9 42

BlackGPIO SW_Vision(GPIO_30, input, FastMode);		// P9 11
BlackGPIO SW_Bundle(GPIO_31, input, FastMode);		// P9 13
BlackGPIO SW_Power(GPIO_48, input, FastMode);		// P9 15

BlackI2C myI2C(I2C_2, 0x22);
BlackSPI mySpi(SPI0_0, 8, SpiDefault, 200000);


BallHandle		BH_right(P8_13, GPIO_69, GPIO_68, GPIO_46, GPIO_65);	/* pwm, dir, reset, ff1, ff2 */
BallHandle		BH_left(P8_19, GPIO_66, GPIO_67, GPIO_44, GPIO_26);		/* pwm, dir, reset, ff1, ff2 */
OpticalFlow		motion(GPIO_112, GPIO_117, GPIO_115, GPIO_60, &mySpi);	/* ncs, npd, rst, led */

BlackADC		ADC_Light(AIN0);

BlackPWM		ShovelSelect(P9_14);
Shovel			shovel;

timeval			time_now;
timeval			last_ping;

CV				threw[5], cv_main;

bool			ex = false;


#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_ACTUATOR_H_ */
