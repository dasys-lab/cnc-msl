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
	TCCR1A = 0x00;

	TCCR1B = 0x00;
	TCCR1B |= (1<<WGM12) | (1<<CS10);	// Prescaler: 1 & CTC
	char str1[20];
	sprintf(str1, "1");
	debug(str1);

//	TIMSK0 = TIMSK0 & ~0x07;		// TODO: irgendwas stimmt mit dem TIMSK register nicht
	TIMSK1 |= (1<<OCIE1A);	// Compare Match Interrupt Enable  - OCIE0A


	uint8_t ocr = 79;		// ocr = F_CPU/1000000 * TIMER_RES / 2 / TIMER_PRESCALER - 1
	OCR1A = ocr;
	char str[20];
	sprintf(str, "2");
	debug(str);

	message_handler();
	message_handler();
	message_handler();
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
