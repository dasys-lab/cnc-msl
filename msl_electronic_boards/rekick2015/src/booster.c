/*
 * booster.c
 *
 *  Created on: Sep 12, 2016
 *      Author: Carpe Noctem
 */

#include "booster.h"
#include "messages.h"

#include "defaults.h"
#include "global.h"

#include <util/delay.h>
#include <string.h>

enum eState {
	State_Deactivated = 0x00,
	State_Error = 0x01,
	State_Error_CAP_OVLO = 0x02,		// Over Voltage Lock Out
	State_Activated = 0x10,
	State_ActivatedHold = 0x11,
	State_ActivatedKicking = 0x12,
	State_VoltageLow = 0x20,
	State_VoltageLowLogic = 0x21,
	State_VoltageLowBooster = 0x22,
	State_EmergencyShutdown = 0xE0,
	State_EmergencyTriggered = 0xE1,
	State_EmergencyReset = 0xEE,

	// These States should never be reached
	State_False = 0xF0,
	State_FalseKick = 0xF1,
};

/**
 * This structure builds the info message
 *
 * 5 bytes long
 */
struct BOOSTER_INFO {
	enum eState state;					//< the state of the booster
	uint16_t supply_voltage;		//< the adc value of the supply voltage (Volt)
	uint16_t capacitors_voltage;	//< the adc voltage of the capacitors (Volt)
};


volatile uint16_t adc_logic_raw = 0;
volatile uint16_t adc_booster_raw = 0;
volatile uint16_t adc_capacitor_raw = 0;

enum eMode mode;
uint16_t desired_voltage = 330.0;
uint32_t last_heartbeat = 0;


void booster_init()
{
	SET_OUTPUT(RESET_NOTAUS);
	RESET(RESET_NOTAUS);
	SET_INPUT(NOTAUS);
	RESET(NOTAUS);

	SET_OUTPUT(ACTIVATE_BOOSTER);
	RESET(ACTIVATE_BOOSTER);
	SET_OUTPUT(CHARGE);
	RESET(CHARGE);
	SET_INPUT(FAULT);
	RESET(FAULT);
	SET_INPUT(DONE);
	RESET(DONE);

	booster_reset();
	mode = Mode_Automatic;
}

void booster_reset()
{
	RESET(NOTAUS);
	RESET(ACTIVATE_BOOSTER);
	RESET(CHARGE);
	RESET(FAULT);
	RESET(DONE);
	RESET(KICK);


	SET(RESET_NOTAUS);
	_delay_ms(1);
	RESET(RESET_NOTAUS);
}

enum eState booster_getState()
{
	if(!IS_SET(NOTAUS))
		return State_EmergencyShutdown;

	if(IS_SET(RESET_NOTAUS))
		return State_EmergencyReset;

	switch(mode) {
		case Mode_Error:
			return State_Error;
			break;

		case Mode_ErrorCap:
			return State_Error_CAP_OVLO;
			break;
	}

	bool active = IS_SET(ACTIVATE_BOOSTER);
	bool kick = IS_SET(KICK);
	bool fault = IS_SET(FAULT);
	bool charge = IS_SET(CHARGE);

	if(fault)
	{
		uint16_t logicVoltage = booster_getLogicVoltage();
		uint16_t boosterVoltage = booster_getBoosterVoltage();

		if(logicVoltage < 18 || logicVoltage > 29)
			return State_VoltageLowLogic;

		if(boosterVoltage < 18 || logicVoltage > 29)
			return State_VoltageLowBooster;

		return State_VoltageLow;
	}

	if(active != charge) // Should be the same. Otherwise emergency shutdown is triggered
		return State_EmergencyTriggered;

	if(!active & !kick) {
		if (mode == Mode_SoftwareControlledHold)
			return State_ActivatedHold;
		else
			return State_Deactivated;
	}

	if(active & !kick)
		return State_Activated;

	if(!active & kick)
		return State_ActivatedKicking;

	if(active & kick)		// Should never be reached
		return State_FalseKick;

	return State_False;
}

int8_t booster_activate()
{
	switch (booster_getState())
	{
		case State_Activated:
		case State_ActivatedHold:
			SET(ACTIVATE_BOOSTER);
			return 1;
			break;

		case State_Deactivated:
			SET(ACTIVATE_BOOSTER);
			return 1;
			break;

		case State_ActivatedKicking:
			RESET(ACTIVATE_BOOSTER);
			return 0;
			break;

		default:
			error("Booster not Activated, check States");
			booster_deactivate();
			return 0;
			break;
	}

	return 1;
}

void booster_deactivate()
{
	RESET(ACTIVATE_BOOSTER);
	RESET(KICK);
}

uint8_t booster_canKick() {
	switch (booster_getState()) {
		case State_Activated:
		case State_ActivatedHold:
			return 1;
			break;

		case State_ActivatedKicking:
			booster_reset();
			debug("Asked for Kick while Kicking");
			return 0;
			break;

		default:
			return 0;
			break;
	}
}

uint16_t booster_getLogicVoltage()
{
	// factor = ADC-Ref-Voltage * Voltage-Divider / ADC-Resolution
	static double factor = 0.048479; // 5.0 * 556/56 / 1024;

	double ret = adc_logic_raw * factor;

	return (uint16_t) ret; // * factor;
}

uint16_t booster_getBoosterVoltage()
{
	// factor = ADC-Ref-Voltage * Voltage-Divider / ADC-Resolution
	static double factor = 0.048479; // 5.0 * 556/56 / 1024;

	double ret = adc_booster_raw * factor;

	return (uint16_t) ret; // * factor;
}

uint16_t booster_getCapacitorVoltage()
{
	// factor = ADC-Ref-Voltage * Voltage-Divider / ADC-Resolution
	static double factor = 0.328664; // 5.0 * 12587/187 / 1024;

	double ret = adc_capacitor_raw * factor;

	return (uint16_t) ret; // * factor;
}

/**
 * Callback function which prints the actual capacitors message
 */
void booster_printVoltage() {
	char str1[20];
	sprintf(str1, "L: %d, %d V", adc_logic_raw, booster_getLogicVoltage());
	debug(str1);

	char str2[20];
	sprintf(str2, "B: %d, %d V", adc_booster_raw, booster_getBoosterVoltage());
	debug(str2);

	char str3[20];
	sprintf(str3, "C: %d, %d V", adc_capacitor_raw, booster_getCapacitorVoltage());
	debug(str3);
}

void booster_sendInfo() {
	struct BOOSTER_INFO info;

	info.state = booster_getState();
	info.supply_voltage = booster_getBoosterVoltage();
	info.capacitors_voltage = booster_getCapacitorVoltage();

	prepareMsg(CMD_STATE, (uint8_t *)&info, 5, PRIORITY_NORM);
}

void booster_setLogicRawVoltage(uint16_t voltage)
{
	adc_logic_raw = voltage;
}

void booster_setBoosterRawVoltage(uint16_t voltage)
{
	adc_booster_raw = voltage;
}

void booster_setCapacitorRawVoltage(uint16_t voltage)
{
	adc_capacitor_raw = voltage;
}

void booster_setMaxVoltage(uint16_t voltage)
{
	if(voltage > MAX_VOLTAGE)
	{
		error("Voltage to high");
		voltage = MAX_VOLTAGE;
	}

	desired_voltage = voltage;
}

void booster_ctrl()
{
	uint32_t time_now;
	timer_get_ms(&time_now);
	char message[30];

	enum eState state = booster_getState();
	static enum eState state_old;

	bool errorFlag = false;

	switch(state)
	{
		case State_Deactivated:
			break;

		case State_Activated:
		case State_ActivatedHold:
			if (time_now - last_heartbeat < PING_TIMEOUT || mode == Mode_Manual) {
				uint16_t capacitor_voltage = booster_getCapacitorVoltage();
				if (booster_getCapacitorVoltage() > MAX_VOLTAGE + 15) {
					errorFlag = true;
					sprintf(message, "Overvoltage. Software detected.");
					booster_deactivate();
					mode = Mode_ErrorCap;
				}

				if (mode == Mode_SoftwareControlled || mode == Mode_SoftwareControlledHold) {
					if (capacitor_voltage >= desired_voltage + 0.5)
						booster_deactivate();
					else if (capacitor_voltage <= desired_voltage - 0.5)
						booster_activate();
				} else {
					booster_activate();
				}
			} else {
				booster_deactivate();
			}
			break;

		case State_ActivatedKicking:
			sprintf(message, "Kicking");
			break;

		case State_VoltageLow:
		case State_VoltageLowLogic:
		case State_VoltageLowBooster:
			sprintf(message, "Voltage: 0x%02X", state);
			booster_deactivate();
			break;

		case State_Error:
		case State_Error_CAP_OVLO:
			errorFlag = true;
			sprintf(message, ">>Error: 0x%02X", state);
			booster_deactivate();
			break;

		case State_EmergencyTriggered:
		case State_EmergencyShutdown:
			errorFlag = true;
			sprintf(message, ">>ES: 0x%02X", state);
			booster_deactivate();
			break;

		case State_EmergencyReset:
			errorFlag = true;
			sprintf(message, "Reset");
			booster_deactivate();
			break;


		// Should never be reached
		case State_False:
		case State_FalseKick:
			errorFlag = true;
			sprintf(message, "State should not be reached: 0x%02X", state);
			booster_deactivate();
			break;

		default:
			errorFlag = true;
			sprintf(message, "Unknown State");
			booster_deactivate();
			break;
	}


	// Handler that Messages don't Spam on CAN
	static uint32_t time_msgLastSended = 0;
	if(state == state_old) {
		if(time_now - time_msgLastSended > 1000) {
			// Send same Message once every Second
			if(errorFlag)
				error(message);
			else
				debug(message);
			time_msgLastSended = time_now;
		}
	} else {
		if(errorFlag)
			error(message);
		else
			debug(message);
		state_old = state;
		time_msgLastSended = time_now;
	}

}
