#include <avr/interrupt.h>
#include <avr/io.h>

#include "defs.h"

// define the maximum of callback functions (max is 8)
#define MAX_CALLBACKS   2

// Prescaler: valid values are: 1, 8, 64, 256, 1024
// change this also where the timer is enabled
#define PRESCALER	256

#ifndef F_CPU
#	error "ERROR: F_CPU not defined"
#endif

// The frequency in ms 
//#define TIMER_RES	(( PRESCALER * 2 * 256 / F_CPU * 1000 ))
#define TIMER_RES	4.096F

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

/**
 * The ticks since startup
 */
static volatile uint32_t ticks = 0;


/**
 * ISR of Timer 0 is called every TIMER_RES ms.
 */
ISR(TIMER0_OVF_vect) {

	uint8_t i;

	ticks++;

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

	// Timer 0, 8 bit timer
	// Prescaler 256
	TCCR0 = (1 << CS02) | (1 << CS01);

	// enable timer overflow on timer 0
	TIMSK |= (1 << TOIE0); // enable overflow interrupt

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

	if (time_ms < TIMER_RES || time_ms > 255*TIMER_RES) {
		return EXIT_FAILURE;
	}
	cycles = (uint8_t)(time_ms/TIMER_RES);

	for (i = 0; i < MAX_CALLBACKS; i++) {
		if (tlist[i].func == NULL) {
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
			return EXIT_SUCCESS;
		}
	}

	return EXIT_FAILURE;
}

/**
 * Get the ticks since power on.
 *
 * The timestamp gets incremented each TIMER_RES.
 *
 * @returns The timestamp
 */
uint32_t timer_get_ticks(void) {
	
	uint8_t sreg = SREG;
	uint32_t ret = 0;

	cli();
	ret = ticks;
	SREG = sreg;

	return ret;
}

/**
 * Get the miliseconds since power on.
 *
 * @returns The time in ms.
 */
uint32_t timer_get_ms(void) {

	return timer_get_ticks() * TIMER_RES ;
}