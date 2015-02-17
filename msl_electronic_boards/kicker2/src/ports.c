/*
 * $Id$
 *
 *
 * Copyright 2005,2006 Carpe Noctem, Distributed Systems Group,
 * University of Kassel. All right reserved.
 *
 * The code is derived from the software contributed to Carpe Noctem by
 * the Carpe Noctem Team.
 *
 * The code is licensed under the Carpe Noctem Userfriendly BSD-Based
 * License (CNUBBL). Redistribution and use in source and binary forms,
 * with or without modification, are permitted provided that the
 * conditions of the CNUBBL are met.
 *
 * You should have received a copy of the CNUBBL along with this
 * software. The license is also available on our website:
 * http://carpenoctem.das-lab.net/license.txt
 *
 *
 * <description>
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "ports.h"

#define ADCREF	(1 << REFS0)

// Debounce input ports.
//
// ISR which is called every 10 ms.
SIGNAL(SIG_OVERFLOW0) {
	
	static uint8_t ct0, ct1;
	uint8_t i;

	TCNT0 = 56;              // preload timer for 10ms
	
	// debounce input keys
	i = keystate ^ ~PIND;   // check for changed state
	ct0 = ~(ct0 & i);        // reset or count ct0
	ct1 = ct0 ^ (ct1 & i);   // reset or count ct1
	i &= ct0 & ct1;          // count until roll over ?
	keystate ^= i;          // then toggle debounced state
}

void ports_init(void) {

	keystate = 0;

	// LEDs are attached on PB0 (yellow) and PB1 (red)
	PORTB &= ~((1 << PB0) | (1 << PB1));
	DDRB = (1 << DDB0) | (1 << DDB1);

	// PORTD (port direction in) (PD2 up to PD7)
	PORTD = 0x00; // deactivate internal pull-up resistors
	DDRD &= ~((1 << DDD2) | (1 << DDD3) | (1 << DDD4) | (1 << DDD5) | (1 << DDD6) | (1 << DDD7)); 
	
	// Start Timer for key recognition (debouncing)
	// 8bit timer normal operation, prescaler 1024
	TCCR0B = (1 << CS02) | (1 << CS00);
	TIMSK0 = (1 << TOIE0); // activate interrupt

	// Enable ADC, single conversion, scale frequency: clk/128
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

	// select channel 6
	ADMUX = ADCREF | 6;
	
	// start a single conversion
	ADCSRA |= (1 << ADSC);

	// wait for end
	while (ADCSRA & (1 << ADSC));
	
}

/// Take a measurement on the ADC
uint16_t ports_get_adc(uint8_t mux) {

	uint8_t i;
	uint16_t res = 0;
	
	if (mux < 6 || mux > 7)
		return 0;

	// select channel and make a test measurement
	ADMUX = ADCREF | mux;
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	
	// measurement
	for (i = 0; i < 3; i++) {
		ADCSRA |= (1 << ADSC);
		while (ADCSRA & (1 << ADSC));
		res += ADCW;
	}
	res /= 3;

	return res;
}

