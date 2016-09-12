// rekick
//
// Firmware for the Kicking Device 2015/2016
//
//
// Author: Lukas Will <lukas.will@gmail.com>


// set fuse bits with:
// -U lfuse:w:0xff:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m
// 16 MHz external Oscilator, rest default

#include <avr/io.h>
#include <util/delay.h>

#include "defaults.h"
#include "global.h"
#include "messages.h"


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
	communication_init();
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
	SET(ACTIVATE_BOOSTER);
	for(int i = 0; i <= 50; i++)
			_delay_ms(1);

	SET(CHARGE);
	for(int i = 0; i <= 50000; i++)
			_delay_ms(1);

	RESET(CHARGE);
	RESET(ACTIVATE_BOOSTER);
	*/
	
	while(1) {
		message_handler();
		for(int i = 0; i <= 30; i++)
			_delay_ms(1);
	}
	return 0;
}
