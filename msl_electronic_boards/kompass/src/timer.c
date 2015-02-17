/* 
 * Timer functions
 *
 * copyright 2007 Kai Baumgart <kb@zkar.de>
 *
 */


#include <avr/pgmspace.h>
#include <stdint.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "defs.h"

#ifdef __DEBUG_ALL__
#	define __DEBUG_TIMER__
#endif

#ifdef __DEBUG_TIMER__
#	include "serial.h"
#endif

// define the maximum of callback functions (max is 8)
#define MAX_CALLBACKS   6

// the time between the iteration of the timer in ms
// 30 resultes to a frequenzy of 33,3 HZ
#define ITERATION_TIME      30

/**
 * High and low byte of the timestamp.
 *
 * \warning The timer gets an overflow after 18.7 hours
 */
static volatile uint8_t timestamp_l, timestamp_h = 0;

typedef struct _TIMER {
	uint8_t cycles;
	uint8_t counter;
	void (*func)(void);
} TIMER;

/**
 * Here are the notifications saved if a callback function should be
 * triggered.
 */
static volatile uint8_t notify = 0x00;

/**
 * Holds all informations of the registered timers
 */
static volatile TIMER tlist[MAX_CALLBACKS];

// timer is called every ITERATION_TIME
ISR(TIMER1_OVF_vect) {

	uint8_t i;

	// preload timer register
	// ITERATION_TIME = prescaler * (2^register_length - preloader) / F_CPU
	TCNT1 = 5536;

	// count timestamp since start
	if (++timestamp_l == 0) ++timestamp_h;

	// set notifies
	for (i = 0; i < MAX_CALLBACKS; i++) {
		if (tlist[i].cycles > 0 && --tlist[i].counter == 0) {
			notify |= (1 << (i+1));
			tlist[i].counter = tlist[i].cycles;
		}
	}

}

/**
 * Initialize the timer.
 */
void timer_init(void) {

	uint8_t i;

	for (i = 0; i < MAX_CALLBACKS; i++) {
		tlist[i].func = NULL;
		tlist[i].counter = 0;
		tlist[i].cycles = 0;
	}

	// 16bit timer
	TCCR1B = (1 << CS11);
	TIMSK |= (1 << TOIE1); // enable overflow interrupt
	TCNT1 = 5536;

	return;
}

/**
 * Calls the outstanding callbackfunctions.
 *
 * This function has to be called as often as possible.
 */
void timer_trigger_callbacks(void) {

	uint8_t i;
	uint8_t sreg;

	for (i = 0; i < MAX_CALLBACKS; i++) {
		if (notify & (1 << (i+1)) && tlist[i].func != NULL) {
			tlist[i].func();
			sreg = SREG;
			cli();
			notify &= ~(1 << (i+1));
			SREG = sreg;
		}
	}
}

/**
 * Register a timer.
 *
 * @param *fptr A function pointer for the callback function
 * @param time_ms The period in which the callback function should be triggered.
 *                The time has to be in range ITERATION_TIME <= x <= 255*ITERATION_TIME
 * @return Yields EXIT_SUCCESS if the function could be registered. Otherwise
 *                EXIT_FAILURE.
 */
uint8_t timer_register(void (*fptr)(void), uint16_t time_ms) {

	uint8_t i;
	uint8_t cycles;

	if (time_ms < ITERATION_TIME || time_ms > 255*ITERATION_TIME) {
#ifdef __DEBUG_TIMER__
		serial_puts_P("TIMER: The min time for the timer is ITERATION_TIME\n");
		serial_puts_P("TIMER: The max time for the timer is 255*ITERATION_TIME\n");
#endif
		return EXIT_FAILURE;
	}
	cycles = (uint8_t)(time_ms/ITERATION_TIME);

	for (i = 0; i < MAX_CALLBACKS; i++) {
		if (tlist[i].func == NULL) {
#ifdef __DEBUG_TIMER__
			serial_puts_P("TIMER: Adding a function to list\n");
#endif
			tlist[i].func = fptr;
			tlist[i].counter = cycles;
			tlist[i].cycles = cycles; // this at the end
			return EXIT_SUCCESS;
		}
	}

	return EXIT_FAILURE;
}

/**
 * Remove a timer.
 *
 * @param The function pointer which were registered
 * @return EXIT_SUCCESS or EXIT_FAILURE
 */
uint8_t timer_deregister(void (*fptr)(void)) {

	uint8_t i, sreg;

	for (i = 0; i < MAX_CALLBACKS; i++) {
		if (tlist[i].func == fptr) {
			tlist[i].cycles = 0;
			tlist[i].counter = 0;
			tlist[i].func = NULL;
			sreg = SREG;
			cli();
			notify &= ~(1 << (i+1));
			SREG = sreg;
#ifdef __DEBUG_TIMER__
			serial_puts_P("TIMER: Removed a function from list\n");
#endif
			return EXIT_SUCCESS;
		}
	}

	return EXIT_FAILURE;
}


/**
 * Get the timestamp since power on.
 *
 * The timestamp gets incremented each ITERATION_TIME.
 *
 * @returns The timestamp
 */
uint16_t timer_get_timestamp(void) {

	uint16_t ts;
	uint8_t sreg = SREG;

	cli();
	ts =  (timestamp_h << 8) + timestamp_l;
	SREG = sreg;

	return ts;
}

/**
 * Get the miliseconds since power on.
 *
 * @returns The time in ms.
 */
uint32_t timer_get_ms(void) {

	return (((uint32_t)timer_get_timestamp()) * ITERATION_TIME);
}

