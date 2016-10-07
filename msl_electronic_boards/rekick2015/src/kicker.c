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
void kicker_addKickJob(uint16_t us10)
{
	timer_get_ms(&kick_job.timestamp);
	kick_job.release_time = us10;

	return;
}

// save the message
// the kick is done by kicker_task_handler
void kicker_addKickJobForced(uint16_t us10, uint8_t forceVoltage)
{
	uint16_t voltage = booster_getCapacitorVoltage();

	if (forceVoltage > voltage + FORCED_VOLTAGE_RANGE/2 || forceVoltage < voltage - FORCED_VOLTAGE_RANGE/2) {
		warning("Cannot reach this voltage");
		return;
	}

	timer_get_ms(&kick_job.timestamp);
	kick_job.release_time = us10;
	kick_job.at_voltage = forceVoltage;

	return;
}

void kicker_kick(uint16_t us10) {
	booster_deactivate();

	if(us10 > (MAX_KICK_TIME * 100)) {
		us10 = MAX_KICK_TIME * 100;
		char message[30];
		sprintf(message, "Kicktime to long! Set to %dms", MAX_KICK_TIME);
		error(message);
	}

	uint16_t us16 = us10 / 1.6;
	cli();
	kicker_ticks = us16;
	sei();

	SET(KICK);
	while(kicker_ticks > 0);
	RESET(KICK);

	booster_activate();
}

// handle the kick job
// _not_ thread safe
void kicker_ctrl(void) {
	uint8_t sreg;
	uint32_t time_now;
	timer_get_ms(&time_now);

	// check for a job: no job to do if timestamp is 0
	if (kick_job.timestamp == 0)
		return;

	// check time between shots
	if (time_now - kick_job.last_kick < TIME_BETWEEN_TWO_SHOTS) {
		// invalidate data
		kick_job.timestamp = 0;
		return;
	}

	// check for timeout: the job expires after 2 seconds
	if (time_now - kick_job.timestamp > KICK_TASK_EXPIRE) {
		warning("Kick job expired.");
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

	// check if the booster is enabled
	if (!booster_canKick()) {
		debug("Can not kick. Booster is in invalid state.");
		return;
	}

	timer_get_ms(&time_now);
	uint32_t kickdelay = time_now - kick_job.timestamp;

	kicker_kick(kick_job.release_time);

	timer_get_ms(&time_now);
	uint32_t kicktime = time_now - kickdelay;

	// debug time between kicker message and release
	char msg[30];
	sprintf(msg, "Kickdelay: %lu ms", kickdelay);
	debug(msg);
	sprintf(msg, "Kicktime: %lu ms", kicktime);
	debug(msg);

	// everything fine
	kick_job.timestamp = 0;
	timer_get_ms(&kick_job.last_kick);

	return;
}
