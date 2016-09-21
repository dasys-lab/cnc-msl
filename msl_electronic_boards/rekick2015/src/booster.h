/*
 * booster.h
 *
 *  Created on: Sep 12, 2016
 *      Author: Carpe Noctem
 */

#ifndef CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_BOOSTER_H_
#define CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_BOOSTER_H_

#include "global.h"

#include <avr/io.h>

#define State_Deactivated			0x00
#define State_Activated				0x01
#define State_Kicking				0x02
#define State_VoltageLow			0x10
#define State_VoltageLowLogic		0x11
#define State_VoltageLowBooster		0x12
#define State_EmergencyShutdown		0xE0
#define State_TriggeredEmergency	0xE1
#define State_Reset					0xD0

// Should never be reached
#define State_False					0xF0
#define State_FalseKick				0xF1


#define MAX_VOLTAGE					330		// Only change this value, if you have changed Hardware
#define PING_TIMEOUT				1000	// ms


bool software_controled_boost = 0;
double desired_voltage = 330.0;
uint8_t manual_mode = false;
uint32_t last_heartbeat = 0;

volatile uint16_t adc_logic_raw = 0;
volatile uint16_t adc_booster_raw = 0;
volatile uint16_t adc_capacitor_raw = 0;

void booster_init();
void booster_reset();
int8_t booster_getState();
int8_t booster_activate();
void booster_deactivate();
double booster_getLogicVoltage();
double booster_getBoosterVoltage();
double booster_getCapacitorVoltage();
void booster_setMaxVoltage(double voltage);
void booster_ctrl();

#endif /* CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_BOOSTER_H_ */
