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
//#include "messages.h"


int main(void) {
	// 0 - Eingang
	// 1 - Ausgang
	DDRB = 0x00;
	DDRC = 0x00;
	DDRD = 0x00;

	// 0 - 0V / Pull-up deaktiviert
	// 1 - 5V / Pull-up aktiviert
	PORTB = 0x00;
	PORTC = 0x00;
	PORTD = 0x00;

	for(int i = 0; i <= 2000; i++)
		_delay_ms(1);

	// enable interrupts
	sei();

	adc_init();
	communication_init();
	// ETHERNET

	// BOOSTER
	// KICKER

	for(int i = 0; i <= 2000; i++)
		_delay_ms(1);
/*
	SET(RESET_NOTAUS);

	for(int i = 0; i <= 50; i++)
		_delay_ms(1);

	RESET(RESET_NOTAUS);
*/
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
	
	debug("-------");

	while(1) {
		message_handler();
		adc_handler();
		booster_ctrl();
		// kicker_kick_handler();

		static uint8_t test = 0;
		if(++test > 50)
		{
			print_voltage();
		}

		for(int i=0; i<=30; i++)
			_delay_ms(1);
	}
	return 0;
}
