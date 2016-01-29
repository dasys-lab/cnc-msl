#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include "global.h"
#include "defaults.h"
#include "booster.h"
#include "messages.h"
#include "uart.h"
#include "timer.h"
#include "ports.h"
#include "kicker.h"


#define MAX(x,y) ((x)>(y)?(x):(y))
#define MIN(x,y) ((x)<(y)?(x):(y))

// allow a variance from a forced voltage (in Volt)
#define EPSILON_FORCED_VOLTAGE	0

// time in which the kick task expires
#define KICK_TASK_EXPIRE	2000

#define TIME_BETWEEN_TWO_SHOTS	200 // ms

volatile uint16_t pending_us10 = 1;

// called every 10us
ISR(TIMER2_COMP_vect) {
	static uint8_t uscnt = 0;
	//TOGGLE(TEST);
	if (pending_us10 > 0) { // && ++uscnt > 100) {
		//uscnt = 0;
		pending_us10--;

	}
	if (++uscnt > 100) {
		timer_incTimer();
		uscnt = 0;

	}

}

struct KICK_STRUCT {
	uint32_t timestamp;
	uint32_t last_kick;
	uint16_t  release_time;
	uint8_t  at_voltage;
} kick_job = {0, 0, 0, 0};

void kicker_init(void) {

	//debug
	//SET_OUTPUT(TEST);

	// Init PWM
	TIMSK |= (1 << OCIE2);
	TCCR2 |= (1 << WGM21) | (1 << CS20);
	OCR2 = F_CPU/100000;
	
	// configure the port to release the kicker
	SET_OUTPUT(RELEASE);
	RESET(RELEASE);
}

// save the message
// the kick is done by kicker_task_handler
void kicker_add_kick_job(uint16_t us10) {
	kick_job.timestamp = timer_get_ms();
	kick_job.release_time = us10;

	return;
}

// save the message
// the kick is done by kicker_task_handler
void kicker_add_kick_job_forced(uint16_t us10, uint8_t forceVoltage) {
	if (forceVoltage > max_voltage || forceVoltage < max_voltage - 10) {
		warning("Cannot reach this voltage");
		return;
	}

	kick_job.timestamp = timer_get_ms();
	kick_job.release_time = us10;
	kick_job.at_voltage = forceVoltage;

	return;
}

// handle the kick job
// _not_ thread safe
void kicker_kick_handler(void) {
	uint8_t sreg;
	//uint8_t i;
	uint32_t time_now = timer_get_ms();

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
		int16_t delta = (((int16_t) kick_job.at_voltage) - ((int16_t)get_capacitors_voltage()));
		if (abs(delta) > EPSILON_FORCED_VOLTAGE)
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
	if (!booster_can_kick()) {
		debug("Cannot kick. Booster state is disabled.");
		return;
	}
	//kick_job.timestamp = timer_get_ms(); //DEBUGGING
	booster_pwm_disable();
	SET(RELEASE);
	sreg = SREG;
	cli();
	pending_us10 = kick_job.release_time;
	SREG=sreg;
	for(;;) {
		if((pending_us10 <= 0) && ((pending_us10 & 0xFF) == 0)) break; //double check for handling race condition
	}
	RESET(RELEASE);
	OCR1A = 104;
	OCR1B = 32;		
	if (auto_boost)
		booster_pwm_enable();

	// debug time between kicker message and release
	char out[30];
	uint32_t delta = timer_get_ms() - kick_job.timestamp;
	if (delta > 65000)
		delta = 0;
	sprintf(out, "Kicktime: %u ms", (uint16_t)delta);
	debug(out);

	// everything fine
	kick_job.timestamp = 0;
	kick_job.last_kick = timer_get_ms();

	return;
}

void kicker_task_handler(void) {
	kicker_kick_handler();
}

