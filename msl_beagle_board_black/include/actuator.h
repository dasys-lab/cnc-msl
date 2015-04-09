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


BlackGPIO i_magnet(GPIO_26, input, FastMode);		// P8 14
BlackGPIO i_accel(GPIO_27, input, FastMode);		// P8 17
BlackGPIO i_temp(GPIO_65, input, FastMode);			// P8 18
BlackGPIO i_gyro(GPIO_61, input, FastMode);			// P8 26

BlackGPIO LED_Vision(GPIO_48, output, FastMode);	// P9 15
BlackGPIO LED_Bundle(GPIO_49, output, FastMode);	// P9 23
BlackGPIO LED_Power(GPIO_20, output, FastMode);		// P9 41		// Oder GPIO_116

BlackGPIO SW_Vision(GPIO_30, input, FastMode);		// P8 07
BlackGPIO SW_Bundle(GPIO_31, input, FastMode);		// P8 07
BlackGPIO SW_Power(GPIO_31, input, FastMode);		// P8 07

BlackI2C myI2C(I2C_2, 0x22);
BlackSPI mySpi(SPI0_0, 8, SpiDefault, 200000);


BallHandle		BH_right(P8_13, GPIO_67, GPIO_66, GPIO_69, GPIO_68);
BallHandle		BH_left(P8_19, GPIO_44, GPIO_45, GPIO_47, GPIO_46);
OpticalFlow		motion(GPIO_67, GPIO_66, GPIO_69, GPIO_68, &mySpi);	/*ncs, npd, rst, led*/

BlackADC		ADC_Light(AIN1);

BlackPWM		ShovelSelect(P9_14);
Shovel			shovel;

timeval			time_now;
timeval			last_ping;

CV				c_bhl, c_bhr, c_shovel, c_light, c_switches;

bool			ex = false;


#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_ACTUATOR_H_ */
