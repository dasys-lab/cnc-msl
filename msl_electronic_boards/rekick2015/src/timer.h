/*
 * timer.h
 *
 *  Created on: Sep 19, 2016
 *      Author: Carpe Noctem
 */

#ifndef CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_TIMER_H_
#define CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_TIMER_H_

#include <avr/io.h>
#include <avr/interrupt.h>

#define TIMER_RES	16		// in us


volatile uint32_t ticks = 0;
volatile int16_t kicker_ticks_16us = -1;

#endif /* CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_TIMER_H_ */
