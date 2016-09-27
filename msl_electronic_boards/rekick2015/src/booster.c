/*
 * booster.c
 *
 *  Created on: Sep 12, 2016
 *      Author: Carpe Noctem
 */

#include "booster.h"

#include "defaults.h"
#include "global.h"

#include <util/delay.h>
#include <string.h>


volatile uint16_t adc_logic_raw = 0;
volatile uint16_t adc_booster_raw = 0;
volatile uint16_t adc_capacitor_raw = 0;

bool software_controled_boost = 0;
double desired_voltage = 330.0;
uint8_t manual_mode = false;
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

int8_t booster_getState()
{
	if(!IS_SET(NOTAUS))
		return State_EmergencyShutdown;

	if(IS_SET(RESET_NOTAUS))
		return State_Reset;

	bool active = IS_SET(ACTIVATE_BOOSTER);
	bool kick = IS_SET(KICK);
	bool fault = IS_SET(FAULT);
	bool charge = IS_SET(CHARGE);
	// bool done = IS_SET(DONE);		// Not Implemented

	if(fault)
	{
		uint16_t logicVoltage = booster_getLogicVoltage();
		uint16_t boosterVoltage = booster_getBoosterVoltage();

		if(logicVoltage < 9 || logicVoltage > 17)
			return State_VoltageLowLogic;

		if(boosterVoltage < 18 || logicVoltage > 29)
			return State_VoltageLowBooster;

		return State_VoltageLow;
	}

	if(active != charge) // Should be the same. Otherwise emergency shutdown is triggered
		return State_TriggeredEmergency;

	if(!active & !kick)
		return State_Deactivated;

	if(active & !kick)
		return State_Activated;

	if(!active & kick)
		return State_Kicking;

	if(active & kick)		// Should never be reached
		return State_FalseKick;

	return State_False;
}

int8_t booster_activate()
{
	switch (booster_getState())
	{
		case State_Activated:
			debug("Booster already activated.");
		case State_Deactivated:
			SET(ACTIVATE_BOOSTER);
			return 1;
			break;

		case State_Kicking:
			RESET(ACTIVATE_BOOSTER);
			return -1;
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

double booster_getLogicVoltage()
{
	// factor = ADC-Ref-Voltage * Voltage-Divider / ADC-Resolution
	static double factor = 5.0 * 556/56 / 1024;
	double calc = factor * adc_logic_raw;

	return calc;
}

double booster_getBoosterVoltage()
{
	// factor = ADC-Ref-Voltage * Voltage-Divider / ADC-Resolution
	static double factor = 5.0 * 556/56 / 1024;
	double calc = factor * adc_booster_raw;

	return calc;
}

double booster_getCapacitorVoltage()
{
	// factor = ADC-Ref-Voltage * Voltage-Divider / ADC-Resolution
	static double factor = 5.0 * 12587/187 / 1024;
	double calc = 0.32866 * adc_capacitor_raw;//factor * adc_capacitor_raw;

	char str[20];
	sprintf(str, "CAP-V: %.2lf", calc);
	debug(str);

	return calc;
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

void booster_setMaxVoltage(double voltage)
{
	if(voltage > (double) MAX_VOLTAGE)
	{
		error("Voltage to high");
		voltage = (double) MAX_VOLTAGE;
	}

	desired_voltage = voltage;
}

void booster_ctrl()
{
	uint32_t time_now = timer_get_ms();
	char message[30];

	uint8_t state = booster_getState();
	static uint8_t state_old = 0xFF;

	switch(state)
	{
		case State_Deactivated:
			break;

		case State_Activated:
			if (time_now - last_heartbeat < PING_TIMEOUT || manual_mode)
			{
				if (!software_controled_boost)
					return;

				double capacitor_voltage = booster_getCapacitorVoltage();

				if (capacitor_voltage >= desired_voltage + 0.5)
					booster_deactivate();
				else if (capacitor_voltage <= desired_voltage - 0.5)
					booster_activate();
			}
			else {
				booster_deactivate();
			}
			break;

		case State_Kicking:
			sprintf(message, "Kicking");
			break;

		case State_VoltageLow:
			sprintf(message, "Voltage Low");
			booster_deactivate();
			break;

		case State_VoltageLowLogic:
			sprintf(message, "Logic Voltage Low");
			booster_deactivate();
			break;

		case State_VoltageLowBooster:
			sprintf(message, "Booster Voltage Low");
			booster_deactivate();
			break;

		case State_TriggeredEmergency:
			sprintf(message, ">T-ES<");
			booster_deactivate();
			break;

		case State_EmergencyShutdown:
			sprintf(message, ">ES<");
			booster_deactivate();
			break;

		case State_Reset:
			sprintf(message, "Reset");
			booster_deactivate();
			break;


		// Should never be reached
		case State_False:
		case State_FalseKick:
			sprintf(message, "State should not be reached");
			booster_deactivate();
			break;

		default:
			sprintf(message, "Unknown State");
			booster_deactivate();
			break;
	}


	if(state == state_old) {
		static uint32_t last_sended = 0;
		char str[40];
		sprintf(str, "Zeit: %d, ZeitDiff: %d", time_now, time_now - last_sended);
		debug(str);
		if(time_now - last_sended > 1000)	// Send same Message once every Second
		{
			debug(message);
			last_sended = time_now;
		}
	} else {
		debug(message);
		state_old = state;
	}

}
