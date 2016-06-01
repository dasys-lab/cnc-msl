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


	
	SET_OUTPUT(ACTIVATE_BOOSTER);
	RESET(ACTIVATE_BOOSTER);
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

	for(int i = 0; i <= 5000; i++)
		_delay_ms(1);

	SET(ACTIVATE_BOOSTER);

	for(int i = 0; i <= 200; i++)
			_delay_ms(1);

	RESET(ACTIVATE_BOOSTER);
	
	while(1) {

	}
	return 0;
}
