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



/* TODO
 * Verbesserungen für das Layout:
 *
 * - Andere Lastwiderstände (Metaldraht, KEINE Dickschicht)
 * - MCU kann NOTAUS auslösen
 *
 * - MCU kann Laden ein- und ausschalten (unabhängig von activate_booster)
 *
 */

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include "defaults.h"
#include "global.h"
//#include "messages.h"

//#include "booster.h"


//TESTS
#include "timer.h"

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


	// enable interrupts
	sei();

	communication_init();

	adc_init();
	booster_init();
	kicker_init();
	timer_init();
	// ETHERNET

	// BOOSTER
	// KICKER

	while(1) {
		message_handler();
		//adc_handler();
		kicker_ctrl();
		booster_ctrl();



		for(uint8_t i = 0; i<=100; i++) {
			message_handler();
			_delay_us(10);
		}

		static uint16_t cnt_printVoltage = 0;
		if (++cnt_printVoltage >= 1000) {
			booster_printVoltage();
			cnt_printVoltage = 0;
		}
	}
	return 0;
}
