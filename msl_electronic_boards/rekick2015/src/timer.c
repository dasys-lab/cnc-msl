/*
 * timer.c
 *
 *  Created on: Sep 19, 2016
 *      Author: Carpe Noctem
 */

#include "timer.h"


void timer_init(void)
{
	TCCR0A = 0x00;
	TCCR0B = 0x00;
	TCCR0B |= (1<<CS00);	// Prescaler: 1

	TIMSK0 = 0x00;
	TIMSK0 |= (1<<TOIE0);
}

/**
 * Get the ticks since power on.
 *
 * @returns The timestamp
 */
uint32_t timer_get_ticks(void)
{
	uint32_t ret = 0;

	cli();
	ret = ticks;
	sei();

	return ret;
}

/**
 * Get the milliseconds since power on.
 *
 * @returns The time in ms.
 */
uint32_t timer_get_ms(void)
{
	return timer_get_ticks() * TIMER_RES / 1000;
}


ISR(TIMER0_OVF_vect) {
	ticks++;
	if(kicker_ticks_16us > 0)
		kicker_ticks_16us--;
}
