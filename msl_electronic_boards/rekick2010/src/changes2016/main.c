// rekick
//
// Firmware for the rekick controlling device.
//
// This device controls the servo for rotating the coil to the three directions
// and loads the capacitors with use of the boosting device.
//
// Author: Kai Buamgart <kb@zkar.de> (c) 2008
//
// TODO Software:
//  * Servo 1 PWM enablen
//  * Servo 2 PWM enablen
//  * Rotation
//  * Schalter Rotor
//  * Timeout CAN-Messages
//  * Erste CAN-Messages annehmen aber verwerfen
//  * CAN-ID behandeln
//  * ReKick deaktiviert sich selbst, wenn innerhalb einer bestimmten Zeit
//  * keine Alive-Message von Kicker.exe angenommen wurde
//  * ReKick kann in einen manuellen oder debug-Modus geschaltet werden, wo
//  * keine Alive-Messages von einer überliegenden Software benötigt wird.
//  * Logging. Man könnte den EEPROM zum loggen benutzen.
//  * Abzuspeichern wäre: Ladezeit nach einem Schuss, Schuss, etc.
//
// TODO Hardware:
//  * P-MOSFET rekick einbauen und testen
//  * Am Booster Leiterbahnen dicker
//  * Gegentakt-Schaltung wurde verändert. Dies im Layout aufnehmen.

#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <stdio.h>

#include "defs.h"
#include "messages.h"
#include "timer.h"
#include "booster.h"
#include "kicker.h"
#include "ports.h"
#include "defaults.h"
#include <avr/io.h>
#include "global.h"
#include "ports.h"

// prototypes
char *check_hw_reset(unsigned char);

int main(void) {

	//disable booster power (power off)
	SET_OUTPUT(POWER_INV);
	SET_OUTPUT(POWER_NORM);
	RESET(POWER_INV);
	RESET(POWER_NORM);

	// save reset-register and reset the value
	uint8_t reset_msg[1] = {MCUCSR};
	MCUCSR = 0x00;

	// turn off watchdog
	WDTCR = 0x00;
	
	// enable interrupts
	sei();

	can_init();
	ports_init();
	kicker_init();
	timer_init();
	booster_init();

	// wait until eth2can is up and running or erase can_buffer
	while (timer_get_ms() < 1500) {
		clear_receive_buffer();
	}
	// send reset message
	can_put_cmd(CMD_RESET, reset_msg, 1);

	//if (timer_register(toggle_status_led, 500) != EXIT_SUCCESS)
	//	error("Cannot register timer\n");
	
	// to initialize we let the rekick rotate the kicker to position #1
	//kicker_add_rotate_job(1);
	/*char msg[20];
	
	while(1) {
		sprintf(msg, "%lu\n",timer_get_ms());
		debug(msg);

	}*/

	for (;;) {
		timer_trigger_callbacks();
		message_handler();
		kicker_task_handler();
		booster_ctrl();		
	}
	return 0;
}

/**
 * Check which reset flag is set
 *
 * @param mcucsr The register which holds the data.
 * @returns      A string with the output.
 */
char *check_hw_reset(unsigned char mcucsr) {
	if (mcucsr & (1 << PORF))
		return PSTR("Power On");
	else if (mcucsr & (1 << JTRF))
		return PSTR("JTAG");
	else if (mcucsr & (1 << WDRF))
		return PSTR("WATCHDOG");
	else if (mcucsr & (1 << BORF))
		return PSTR("BROWN OUT");
	else if (mcucsr & (1 << EXTRF))
		return PSTR("External");
	else
		return PSTR("UNDEF");
}
