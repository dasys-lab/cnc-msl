/*
 * timer.c
 *
 *  Created on: Sep 19, 2016
 *      Author: Carpe Noctem
 */

#include "timer.h"

#include <avr/interrupt.h>
#include <avr/io.h>

volatile uint8_t ticks[4] = {0,0,0,0};
volatile uint16_t t16 = 0;
volatile uint32_t t32 = 0;
volatile int16_t kicker_ticks = -1;


void timer_init(void)
{

/*	TCCR1A = 0x00;

	TCCR1B = 0x00;
	TCCR1B |= (1<<CS10);	// Prescaler: 1 & CTC


//	TIMSK0 = TIMSK0 & ~0x07;		// TODO: irgendwas stimmt mit dem TIMSK register nicht
	TIMSK1 |= (1<<TOIE1);	// Compare Match Interrupt Enable  - OCIE0A
*/

	TCCR0A = 0x00;

	TCCR0B = 0x00;
	TCCR0B |= (1<<CS00);	// Prescaler: 1
}

/**
 * Get the ticks since power on.
 *
 * @returns The timestamp
 */
uint32_t timer_get_ticks(void)
{
	uint32_t ret = ((uint32_t) ticks[2] << 16) | ((uint16_t) ticks[1] << 8) | ticks[0];

/*	cli();
	ret = ticks;
	sei(); */

	return ret;//ret;
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

ISR( TIMER0_OVF_vect ) {
	// löst alle 16us aus


	ticks[0]++;
	t32++;

	/*if(ticks[0] >= 100) {
		ticks[0] = 0;
		ticks[1]++;
		if(ticks[1] > 100) {
			ticks[1] = 0;
			ticks[2]++;
		}
	}*/

	if(ticks[0] == 0) {
		ticks[1]++;

		if(ticks[1] == 0) {
			debug("ovf 1");
			char str1[20];
			sprintf(str1, "%lu", t32);
			debug(str1);
		}
	}
	t16++;

//	if(kicker_ticks > 0)
//		kicker_ticks--;
}
/*
ISR(TIMER1_COMPA_vect) {
	ticks++;

	if(kicker_ticks > 0)
		kicker_ticks--;
}
*/
