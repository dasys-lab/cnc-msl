// rekick
//
// Firmware for the Kicking Device 2015/2016
//
//
// Author: Lukas Will <lukas.will@gmail.com>

#include <avr/io.h>
#include <util/delay.h>

#include "defaults.h"
#include "global.h"


int main(void) {
	DDRB = 0x00;
	DDRC = 0x00;
	DDRD = 0x00;


	// Configure Output-Pins
	SET_OUTPUT(ACTIVATE_BOOSTER);
	RESET(ACTIVATE_BOOSTER);
	SET_OUTPUT(CHARGE);
	RESET(CHARGE);
	SET_OUTPUT(ACTIVATE_SERVO);
	RESET(ACTIVATE_SERVO);
	SET_OUTPUT(KICK);
	RESET(KICK);
	SET_OUTPUT(RESET_NOTAUS);
	RESET(RESET_NOTAUS);

	// enable interrupts
	// sei();

	// ADC
	// CAN
	// ETHERNET

	// BOOSTER
	// KICKER

	for(int i = 0; i <= 2000; i++)
		_delay_ms(1);

	SET(RESET_NOTAUS);

	for(int i = 0; i <= 50; i++)
		_delay_ms(1);

	RESET(RESET_NOTAUS);

	for(int i = 0; i <= 2000; i++)
		_delay_ms(1);
/*
	TOGGLE(KICK);
	for(int i = 0; i <= 200; i++)
		_delay_ms(1);
	TOGGLE(KICK);

	for(int i = 0; i <= 200; i++)
		_delay_ms(1);
*/

	SET(ACTIVATE_BOOSTER);

	for(int i = 0; i <= 50; i++)
			_delay_ms(1);

	SET(CHARGE);

	for(int i = 0; i <= 50000; i++)
			_delay_ms(1);

	RESET(CHARGE);


	RESET(ACTIVATE_BOOSTER);
	
	while(1) {
		for(int i = 0; i <= 1000; i++)
			_delay_ms(1);
	}
	return 0;
}
