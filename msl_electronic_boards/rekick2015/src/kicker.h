/*
 * kicker.h
 *
 *  Created on: Sep 19, 2016
 *      Author: Carpe Noctem
 */

#ifndef CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_KICKER_H_
#define CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_KICKER_H_

#include "global.h"

#include <avr/io.h>

#define FORCED_VOLTAGE_RANGE	10		// allow a variance from a forced voltage (in Volt)
#define KICK_TASK_EXPIRE		2000	// time in which the kick task expires
#define TIME_BETWEEN_TWO_SHOTS	200		// ms

struct KICK_STRUCT {
	uint32_t timestamp;
	uint32_t last_kick;
	uint16_t  release_time;
	uint8_t  at_voltage;
} kick_job = {0, 0, 0, 0};

#endif /* CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_KICKER_H_ */
