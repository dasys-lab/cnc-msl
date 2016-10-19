/*
 * timer.c
 *
 *  Created on: Sep 19, 2016
 *      Author: Carpe Noctem
 */

#include "timer.h"

#include <avr/interrupt.h>
#include <avr/io.h>

volatile uint32_t ticks = 0;
volatile int16_t kicker_ticks = -1;


void timer_init(void)
{
	// Timer 0
	TCCR0A = 0x00;

	TCCR0B = 0x00;
	TCCR0B |= (1<<CS00);	// Prescaler: 1

	TIMSK0 = (1<<TOIE0);	// Interrupt bei Overflow
}

/**
 * Get the ticks since power on.
 *
 * @returns The ticks since power on
 */
void timer_get_ticks(uint32_t* ret)
{
	cli();
	*ret = ticks;
	sei();
}

/**
 * Get the milliseconds since power on.
 *
 * @returns The time in ms.
 */
void timer_get_ms(uint32_t* ms)
{
	timer_get_ticks(ms);
	*ms = *ms * TIMER_RES / 1000;
}

ISR(TIMER0_OVF_vect) {
	// löst alle 16us aus
	ticks++;

	if(kicker_ticks > 0)
		kicker_ticks--;
}
