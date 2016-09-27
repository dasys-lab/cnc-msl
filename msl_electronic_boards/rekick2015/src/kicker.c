/*
 * kicker.c
 *
 *  Created on: Sep 19, 2016
 *      Author: Carpe Noctem
 */

#include "defaults.h"
#include "global.h"
#include "kicker.h"
#include "timer.h"

#include <avr/interrupt.h>
#include <util/delay.h>


struct KICK_STRUCT {
	uint32_t timestamp;
	uint32_t last_kick;
	uint16_t  release_time;
	uint8_t  at_voltage;
} kick_job = {0, 0, 0, 0};


void kicker_init(void)
{
	SET_OUTPUT(KICK);
	RESET(KICK);
}

// save the message
// the kick is done by kicker_task_handler
void kicker_add_kick_job(uint16_t us10)
{
	kick_job.timestamp = timer_get_ms();
	kick_job.release_time = us10;

	return;
}

// save the message
// the kick is done by kicker_task_handler
void kicker_add_kick_job_forced(uint16_t us10, uint8_t forceVoltage)
{
	uint16_t voltage = booster_getCapacitorVoltage();

	if (forceVoltage > voltage + FORCED_VOLTAGE_RANGE/2 || forceVoltage < voltage - FORCED_VOLTAGE_RANGE/2) {
		warning("Cannot reach this voltage");
		return;
	}

	kick_job.timestamp = timer_get_ms();
	kick_job.release_time = us10;
	kick_job.at_voltage = forceVoltage;

	return;
}

void kicker_kick(uint16_t us10)
{
	booster_deactivate();

	cli();
	kicker_ticks = us10;
	sei();

	SET(KICK);
	while(kicker_ticks > 0);
	RESET(KICK);

	booster_activate();
}

// handle the kick job
// _not_ thread safe
void kicker_kick_handler(void) {
	uint8_t sreg;
	uint32_t time_now = 0;//timer_get_ms();

	// no job to do if timestamp is 0
	if (kick_job.timestamp == 0)
		return;

	// time between shots
	if (time_now - kick_job.last_kick < TIME_BETWEEN_TWO_SHOTS) {
		// invalidate data
		kick_job.timestamp = 0;
		return;
	}

	// handle forced_voltage
	if (kick_job.at_voltage > 0) {
		int16_t delta = ((int16_t) kick_job.at_voltage) - ((int16_t) booster_getCapacitorVoltage());
		if (abs(delta) > FORCED_VOLTAGE_RANGE/2)
			return;
		kick_job.at_voltage = 0;
	}

	// the job expires after some milliseconds
	if (time_now - kick_job.timestamp > KICK_TASK_EXPIRE) {
		warning("Kick job expired.");
		kick_job.timestamp = 0;
		return;
	}

	// check if the booster is enabled
	/*if (!booster_can_kick()) {
		debug("Cannot kick. Booster state is disabled.");
		return;
	}*/

	uint32_t kickdelay = timer_get_ms() - kick_job.timestamp;
	kicker_kick(kick_job.release_time);
	uint32_t kicktime = timer_get_ms() - kickdelay;


	//TODO: kickerbenachrichtigung

	// debug time between kicker message and release
	char out[30];
	sprintf(out, "Kicktime: %u ms", (uint16_t)kicktime);
	debug(out);

	// everything fine
	kick_job.timestamp = 0;
	kick_job.last_kick = timer_get_ms();

	return;
}
