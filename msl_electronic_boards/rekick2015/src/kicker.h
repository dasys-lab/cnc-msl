/*
 * kicker.h
 *
 *  Created on: Sep 19, 2016
 *      Author: Carpe Noctem
 */

#ifndef CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_KICKER_H_
#define CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_KICKER_H_

//#include "global.h"

#include <avr/io.h>

#define FORCED_VOLTAGE_RANGE	10		// allow a variance from a forced voltage (in Volt)
#define KICK_TASK_EXPIRE		2000	// time in which the kick task expires
#define TIME_BETWEEN_TWO_SHOTS	200		// ms

void kicker_init();
void kicker_add_kick_job(uint16_t us10);
void kicker_add_kick_job_forced(uint16_t us10, uint8_t forceVoltage);
void kicker_kick(uint16_t us10);
void kicker_kick_handler();

#endif /* CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_KICKER_H_ */
