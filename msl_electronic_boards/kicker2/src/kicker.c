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
 
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "kicker.h"
#include "ports.h"
#include "helper.h"

// DEBUG
//#include <stdio.h>
//#include <string.h>
//#include <avr/pgmspace.h>
//#include "uart.h"
// DEBUG

#define VALVE_PUSH          PC1
#define VALVE_PUSH_PORT     PORTC
#define VALVE_PULL          PC0
#define VALVE_PULL_PORT     PORTC
#define PWM_SERVO           OCR1B
#define ROTOR_SWITCH        PIND0
#define ROTOR_SWITCH_PORT   PORTD
#define VALVE_UPPER_EXT			PC3
#define VALVE_UPPER_EXT_PORT	PORTC
#define VALVE_LEFT_EXT			PC5
#define VALVE_LEFT_EXT_PORT	PORTC
#define VALVE_RIGHT_EXT			PC2
#define VALVE_RIGHT_EXT_PORT	PORTC

// durations of extension
uint16_t counter_upper;
uint16_t counter_left;
uint16_t counter_right;

// system time
uint32_t counter_timestamp;

#define DEFAULT_EXTENSION_MAX_OUT 1000
#define DEFAULT_EXTENSION_MIN_SLEEP 4000

uint16_t ext_max_out;
uint16_t ext_min_sleep;

// flag to tell if extensions were ever used / ext_last_use timestamp is valid
uint8_t ext_used;

// timestamp of last retract
uint32_t ext_last_use;

void __inline__ delay(uint32_t ms) {

	uint32_t i;

	for (i=0; i < ms; i++)
		_delay_ms(1);
}


// ==== globals (private) ====
uint16_t pulse_width[3];

// ==== functions ====

ISR(TIMER0_COMPA_vect){

	counter_timestamp++;

	// decrement and test for transition!
	if((counter_upper > 0) && (--counter_upper == 0)) {
	
		VALVE_UPPER_EXT_PORT &= ~(1 << VALVE_UPPER_EXT);
		
		ext_last_use = counter_timestamp;
		ext_used = 1;
		
	}

	if((counter_left > 0) && (--counter_left == 0)) {
	
		VALVE_LEFT_EXT_PORT &= ~(1 << VALVE_LEFT_EXT);
		
		ext_last_use = counter_timestamp;
		ext_used = 1;
		
	}
			
	if((counter_right > 0) && (--counter_right == 0)) {
	
		VALVE_RIGHT_EXT_PORT &= ~(1 << VALVE_RIGHT_EXT);
		
		ext_last_use = counter_timestamp;
		ext_used = 1;
		
	}
	
}

uint8_t kicker_check_time_delta(uint32_t timestamp, uint16_t min_delta) {

	uint32_t current;
	uint32_t current_delta;
	
	// latch timestamp
	current = counter_timestamp;
	
	current_delta = current - timestamp;
	
	return (current_delta >= min_delta);

}

/// @brief Reset kicker and give variables a usable value.
void kicker_init(void) {

	kicker_set_pulse_width(POS_LEFT, 1250);
	kicker_set_pulse_width(POS_MIDDLE, 1875);
	kicker_set_pulse_width(POS_RIGHT, 2500);

	// relays are attached on PC0 up to PC5
	PORTC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3 | (1 << PC4)) | (1 << PC5));
	DDRC |= (1 << DDC0) | (1 << DDC1) | (1 << DDC2) | (1 << DDC3) | (1 << DDC4) | (1 << DDC5);
	
	// Servo is attached on PB2
	DDRB = (1 << DDB2);

	kicker_softreset();

	// Start PWM
	// Phase Correct, TOP is OCR1A, Update OCR1x at TOP, prescaler 8
	// Clear OC1B on Compare Match when upcounting. Set OC1B on Compare
	// Match when downcounting.
	TCCR1A = (1 << COM1B1) | (1 << WGM11) | (1 << WGM10);
	TCCR1B = (1 << WGM13) | (1 << CS11);
	OCR1A = 25000;								// set period to 20 ms
	OCR1B = pulse_width[POS_MIDDLE];


	TCCR0A = _BV(WGM01);
 	TCCR0B = _BV(CS00) | _BV(CS02);
 	OCR0A = 17; 
 	TIMSK0 = _BV(OCIE0A);

	counter_upper = 0;
	counter_left = 0;
	counter_right = 0;
	counter_timestamp = 0;

	ext_min_sleep = DEFAULT_EXTENSION_MIN_SLEEP;
	ext_max_out = DEFAULT_EXTENSION_MAX_OUT;

	ext_used = 0;

	sei();

	return;
}

/// @brief Put the kicker in a defined state.
void kicker_softreset(void) {

	VALVE_PUSH_PORT &= ~(1 << VALVE_PUSH);
	VALVE_PULL_PORT |= (1 << VALVE_PULL);
	delay(10);
	VALVE_PULL_PORT &= ~(1 << VALVE_PULL);

	kicker_rotate(POS_MIDDLE);

	return;
}

uint8_t kicker_rotate(uint8_t num) {

	uint16_t old_width = PWM_SERVO;
	uint16_t delay_ms = 0;
	
	PWM_SERVO = pulse_width[num-1];
	// what a hack
	delay_ms = abs(old_width - PWM_SERVO) * 10 / (abs(pulse_width[POS_LEFT] - pulse_width[POS_RIGHT]) / 100) * 1;
	
	delay(delay_ms);

	return EXIT_SUCCESS;
}
	
// @brief Set the pulse timing for the PWM-channel
uint8_t kicker_set_pulse_width(uint8_t pos, uint16_t val) {
	if (PWM_SERVO == pulse_width[pos]) {
		PWM_SERVO = val;
		delay(1000);
	}
	pulse_width[pos] = val;

	return EXIT_SUCCESS;
}

// @brief Get the actual pulse timing for the PWM-hannel
uint16_t kicker_get_pulse_width(uint8_t pos) {

	return pulse_width[pos];
}

uint8_t kicker_shoot(uint8_t pow) {

	// key pressed: ~keystate & (1 << PINC0)
	// key unpressed: keystate & (1 << PINC0)
	if (~keystate & (1 << ROTOR_SWITCH)) {
		
		/*
		VALVE_PUSH_PORT |= (1 << VALVE_PUSH);
		if (pow > 100) {
			delay(300);
		}
		else {
			delay((uint32_t)(pow));
		}
		VALVE_PUSH_PORT &= ~(1 << VALVE_PUSH);
		
		VALVE_PULL_PORT |= (1 << VALVE_PULL);
		delay(100);
		VALVE_PULL_PORT &= ~(1 << VALVE_PULL);
		*/
		
		VALVE_PUSH_PORT |= (1 << VALVE_PUSH);
		delay((uint32_t)(pow));
		VALVE_PUSH_PORT &= ~(1 << VALVE_PUSH);
		VALVE_PULL_PORT |= (1 << VALVE_PULL);
		delay(50);
		VALVE_PULL_PORT &= ~(1 << VALVE_PULL);
		
		
		return EXIT_SUCCESS;
	} else {
		return EXIT_FAILURE;
	}
}

// tells if any extension is currently active
uint8_t kicker_ext_active() {

	return ((counter_upper | counter_left | counter_right) != 0);

}

uint8_t kicker_uext(uint16_t pow) {

	if(!kicker_ext_active()) {

		if(ext_used == 0 || kicker_check_time_delta(ext_last_use, ext_min_sleep)) {

			if(pow > ext_max_out) {
			
				pow = ext_max_out;
				
			}

			VALVE_UPPER_EXT_PORT |= (1 << VALVE_UPPER_EXT);
			
			counter_upper = pow;
		
			return EXIT_SUCCESS;
			
		}

	}
	
	return EXIT_IGNORE;
		
}

uint8_t kicker_lext(uint16_t pow) {

	if(!kicker_ext_active()) {

		if(ext_used == 0 || kicker_check_time_delta(ext_last_use, ext_min_sleep)) {
		
			if(pow > ext_max_out) {
			
				pow = ext_max_out;
				
			}

			VALVE_LEFT_EXT_PORT |= (1 << VALVE_LEFT_EXT);
			
			counter_left = pow;
			
			return EXIT_SUCCESS;
			
		}
		
	}

	return EXIT_IGNORE;
		
}


uint8_t kicker_rext(uint16_t pow) {

	if(!kicker_ext_active()) {
	
		if(ext_used == 0 || kicker_check_time_delta(ext_last_use, ext_min_sleep)) {

			if(pow > ext_max_out) {
			
				pow = ext_max_out;
				
			}

			VALVE_RIGHT_EXT_PORT |= (1 << VALVE_RIGHT_EXT);
			
			counter_right = pow;
			
			return EXIT_SUCCESS;
		}
	
	}

	return EXIT_IGNORE;
		
}






