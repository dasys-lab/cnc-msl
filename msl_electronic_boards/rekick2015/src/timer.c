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
	TCCR0A = 0x00;
	TCCR0A |= (1<<WGM01);	// CTC-Mode
	TCCR0B = 0x00;
	TCCR0B |= (1<<CS00);	// Prescaler: 1

	TIMSK0 = TIMSK0 & ~0x07;		// TODO: irgendwas stimmt mit dem TIMSK register nicht
	TIMSK0 |= (1<<OCIE0A);	// Compare Match Interrupt Enable


	uint8_t ocr = 79;		// ocr = F_CPU/1000000 * TIMER_RES / 2 / TIMER_PRESCALER - 1
	OCR0A = ocr;
	char str[20];
	sprintf(str, "OCR0A: %d", ocr);
	debug(str);
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


ISR(TIMER0_COMPA_vect) {
	ticks++;

	if(kicker_ticks > 0)
		kicker_ticks--;
}
