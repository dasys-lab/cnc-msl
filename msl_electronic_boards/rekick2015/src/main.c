// rekick
//
// Firmware for the Kicking Device 2015/2016
//
//
// Author: Lukas Will <lukas.will@gmail.com>


// set fuse bits with:
// -U lfuse:w:0xff:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m
// 16 MHz external Oscilator, rest default

/*
iface can0 inet manual
address 127.42.23.180
netmask 255.255.255.0
##ideally use this:
pre-up ip link set can0 up txqueuelen 1000 type can bitrate 1000000
##use this if dmesg show error 'no bittiming available' (kernel flag)
#up ip link set can0 up txqueuelen 1000 type can tq 125 prop-seg 2 phase-seg1 3 phase-seg2 2 sjw 1
##Taker Fix: adding pre-up instead of up, working on nase
#pre-up ip link set can0 type can bitrate 1000000 triple-sampling on
#down ifconfig $IFACE down

## Lukas Test für neue Kickerplatine
#ip link set can0 type can bitrate 125000 triple-sampling on

auto can0
*/

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

	SET_OUTPUT(CAN_TX);
	RESET(CAN_TX);

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
