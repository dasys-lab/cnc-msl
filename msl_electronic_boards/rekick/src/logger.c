#include <avr/eeprom.h>

#include "timer.h"

uint8_t loglevel = LOG_WARNING;
uint8_t debuglevel = LOG_WARNING;

void debug_if(uint8_t thislevel, char *str) {

	if (thislevel <= loglevel) {
		debug(str);
	}

	return;
}

void log_if(uint8_t thislevel, char *str) {

	if (thislevel <= loglevel) {
		// write to eeprom
	}

	return;
}
