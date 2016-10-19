/*
 * booster.h
 *
 *  Created on: Sep 12, 2016
 *      Author: Carpe Noctem
 */

#ifndef CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_BOOSTER_H_
#define CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_BOOSTER_H_

//#include "global.h"

#include <avr/io.h>

#define MAX_VOLTAGE					330		// Only change this value, if you have changed Hardware
#define PING_TIMEOUT				1000	// ms

enum eMode {
	Mode_Automatic,
	Mode_SoftwareControlled,
	Mode_SoftwareControlledHold,
	Mode_Manual,
	Mode_Stop,
	Mode_Error,
	Mode_ErrorCap,
};

extern volatile uint16_t adc_logic_raw;
extern volatile uint16_t adc_booster_raw;
extern volatile uint16_t adc_capacitor_raw;

extern enum eMode mode;
extern uint32_t last_heartbeat;

void booster_init();
void booster_reset();
enum eState booster_getState();
int8_t booster_activate();
void booster_deactivate();
uint16_t booster_getLogicVoltage();
uint16_t booster_getBoosterVoltage();
uint16_t booster_getCapacitorVoltage();
void booster_printVoltage();
void booster_sendInfo();
void booster_setMaxVoltage(uint16_t voltage);
void booster_ctrl();

#endif /* CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_BOOSTER_H_ */
