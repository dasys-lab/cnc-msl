/*
 * $Id: parser.c 3156 2008-03-12 15:09:28Z cn $
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
#include <string.h>
#include <stdbool.h>
#include <avr/pgmspace.h>

#include "parser.h"
#include "ports.h"
#include "kicker.h"
#include "helper.h"
#include "uart.h"

#define BUFFERLENGTH	56

// version number
#define MAJOR	2
#define MINOR	1.5

#define POW_12V		7
#define POW_24V		6

// globals
bool echo = true;

typedef enum {
	CMD_OK = 0,
	CMD_PONG = 1,
	CMD_ERROR = 2,
	CMD_IGNORE = 3,
} RETVAL_CMD;

typedef struct {
	RETVAL_CMD num;
	const char* string;
} PARSER_RET;

//prototypes
PARSER_RET process_cmd(char*, uint8_t);
PARSER_RET process_cmd_set(char*, uint8_t);
PARSER_RET process_cmd_get(char*, uint8_t);
PARSER_RET process_cmd_rotate(char*, uint8_t);
PARSER_RET process_cmd_kick(char*, uint8_t);
PARSER_RET process_cmd_uext(char*, uint8_t);
const char* get_version(void);
PARSER_RET process_cmd_rext(char*, uint8_t);
PARSER_RET process_cmd_lext(char*, uint8_t);

void parser_process_uart(void) {
	
	unsigned int c;
	static char recbuf[BUFFERLENGTH];
	static uint8_t len = 0;
	static bool overflow = false;
	PARSER_RET retval;

	c = uart_getc();

	if (c & UART_NO_DATA) {
		return;
	}

	// check for error
	if (c & UART_FRAME_ERROR) {
		uart_puts_P("ERROR: UART Frame Error\n");
		return;
	}
	if (c & UART_OVERRUN_ERROR) {
		uart_puts_P("ERROR: UART Overrun Error\n");
		return;
	}
	if (c & UART_BUFFER_OVERFLOW) {
		uart_puts_P("ERROR: Buffer overflow error\n");
		return;
	}

	// echo back the received char
	if (echo) {
		uart_putc((unsigned char)c);
	}

	// avoid inserting NULL-Byte in strings
	if (c == 0x00)
		return;

	if (c == '\n' || c == '\r') {
		if (overflow) {
			uart_puts_P("ERROR: PARSER Overflow Error. String is too long.\n");
			overflow = false;
			len = 0;
			return;
		}
		if (len == 0)
			return;
		// add trailing NULL-byte
		recbuf[len] = 0x00;
		retval = process_cmd(recbuf, len);
		len = 0;
		
		if (retval.num == CMD_OK) {
			uart_puts_P("OK");
		} else if (retval.num == CMD_PONG) {
			uart_puts_P("PONG");
		} else if (retval.num == CMD_IGNORE) {
			//uart_puts_P("IGNORE");
			uart_puts_P("OK");
		} else {
			uart_puts_P("ERROR");
		}
		
		if (retval.string != NULL) {
			uart_puts_P(": ");
			uart_puts(retval.string);
		}
		
		uart_puts_P("\n");
	}
	else {
		if (len >= BUFFERLENGTH-1) {
			overflow = true;
			return;
		}
		else {
			recbuf[len++] = c;
		}
	}

}

// be sure, that the string is NULL-terminated
PARSER_RET process_cmd(char* buf, uint8_t len) {

	if ((len > 4) && (strncasecmp(buf, "SET ", 4) == 0)) {
		return process_cmd_set(&buf[4], len-4);
	}
	else if ((len > 4) && (strncasecmp(buf, "GET ", 4) == 0)) {
		return process_cmd_get(&buf[4], len-4);
	}
	else if ((len > 7) && (strncasecmp(buf, "ROTATE ", 7) == 0)) {
		return process_cmd_rotate(&buf[7], len-7);
	}
	else if ((len > 5) && (strncasecmp(buf, "KICK ", 5) == 0)) {
		return process_cmd_kick(&buf[5], len-5);
	}
	else if ((len > 5) && (strncasecmp(buf, "UEXT ", 5) == 0)) {
		return process_cmd_uext(&buf[5], len-5);
	}
	else if ((len > 5) && (strncasecmp(buf, "LEXT ", 5) == 0)) {
		return process_cmd_lext(&buf[5], len-5);
	}
	else if ((len > 5) && (strncasecmp(buf, "REXT ", 5) == 0)) {
		return process_cmd_rext(&buf[5], len-5);
	}
	else if ((len == 4) && (strncasecmp(buf, "PING", 4) == 0)) {
		return (PARSER_RET){CMD_PONG, NULL};
	}
	else {
		return (PARSER_RET){CMD_ERROR, "Command not implemented"};
	}
}

PARSER_RET process_cmd_set(char* buf, uint8_t len) {

	int16_t l;

	if ((len > 19) && (strncasecmp(buf, "Pulse Width Middle ", 19) == 0)) {
		l = atol(&buf[19]);
		if (l < 0)
			return (PARSER_RET){CMD_ERROR, "Invalid parameter"};
		
		kicker_set_pulse_width(POS_MIDDLE, l);
		return (PARSER_RET){CMD_OK, 0x00};
	}
	else if ((len > 18) && (strncasecmp(buf, "Pulse Width Right ", 18) == 0)) {
		l = atol(&buf[18]);
		if (l < 0)
			return (PARSER_RET){CMD_ERROR, "Invalid parameter"};
		
		kicker_set_pulse_width(POS_RIGHT, l);
		return (PARSER_RET){CMD_OK, 0x00};
	}
	else if ((len > 17) && (strncasecmp(buf, "Pulse Width Left ", 17) == 0)) {
		l = atol(&buf[17]);
		if (l < 0)
			return (PARSER_RET){CMD_ERROR, "Invalid parameter"};
		
		kicker_set_pulse_width(POS_LEFT, l);
		return (PARSER_RET){CMD_OK, 0x00};
	}
	else if ((len == 8) && (strncasecmp(buf, "echo off", 8) == 0)) {
		echo = false;
		return (PARSER_RET){CMD_OK, 0x00};
	}
	else if ((len == 7) && (strncasecmp(buf, "echo on", 7) == 0)) {
		echo = true;
		return (PARSER_RET){CMD_OK, 0x00};
	}
	else if ((len > 12) && (strncasecmp(buf, "Ext Max Out ", 12) == 0)) {
		l = atol(&buf[12]);

		if(l < 0) {
			return (PARSER_RET){CMD_ERROR, "Invalid parameter"};
		} else {
			ext_max_out = l;
			return (PARSER_RET){CMD_OK, 0x00};
		}
	}
	else if ((len > 14) && (strncasecmp(buf, "Ext Min Sleep ", 14) == 0)) {
		l = atol(&buf[14]);

		if(l < 0) {
			return (PARSER_RET){CMD_ERROR, "Invalid parameter"};
		} else {
			ext_min_sleep = l;
			return (PARSER_RET){CMD_OK, 0x00};
		}
	}
	else {
		return (PARSER_RET){CMD_ERROR, "Invalid parameter name"};
	}
}

PARSER_RET process_cmd_get(char* buf, uint8_t len) {
	
	uint16_t l;
	char tmp[7];
	char string[15];
	string[0] = 0x00;
	
	if ((len >= 7) && (strncasecmp(buf, "VERSION", 7) == 0)) {
		return (PARSER_RET){CMD_OK, get_version()};
	}
	else if ((len >= 18) && (strncasecmp(buf, "Pulse Width Middle", 18) == 0)) {
		l = kicker_get_pulse_width(POS_MIDDLE);
		utoa(l, tmp, 10);
		return (PARSER_RET){CMD_OK, tmp};
	}	
	else if ((len >= 17) && (strncasecmp(buf, "Pulse Width Right", 17) == 0)) {
		l = kicker_get_pulse_width(POS_RIGHT);
		utoa(l, tmp, 10);
		return (PARSER_RET){CMD_OK, tmp};
	}	
	else if ((len >= 16) && (strncasecmp(buf, "Pulse Width Left", 16) == 0)) {
		l = kicker_get_pulse_width(POS_LEFT);
		utoa(l, tmp, 10);
		return (PARSER_RET){CMD_OK, tmp};
	}
	else if ((len == 5) && (strncasecmp(buf, "Power", 5) == 0)) {
		l = ports_get_adc(POW_12V);
		utoa(l, tmp, 10);
		strcat(string, tmp);
		strcat(string, " ");
		l = ports_get_adc(POW_24V);
		utoa(l, tmp, 10);
		strcat(string, tmp);
		return (PARSER_RET){CMD_OK, string};
	}
	else {
		return (PARSER_RET){CMD_ERROR, "Invalid parameter"};
	}
}

PARSER_RET process_cmd_rotate(char* buf, uint8_t len) {

	uint8_t i;
	
	if (len != 1)
		return (PARSER_RET){CMD_ERROR, "Invalid parameter"};
	
	i = atoi(buf);
	if ((i < 1) || (i > 3)) {
		return (PARSER_RET){CMD_ERROR, "Invalid parameter"};
	}
	else {
		if ((kicker_rotate(i) != EXIT_SUCCESS)) {
			return (PARSER_RET){CMD_ERROR, "Cannot rotate"};
		}
	}

	return (PARSER_RET){CMD_OK, 0x00};
}

PARSER_RET process_cmd_kick(char* buf, uint8_t len) {

	uint8_t i;

	if ((len < 1) || (len > 3)) {
		return (PARSER_RET){CMD_ERROR, "Invalid parameter"};
	}

	i = atoi(buf);
	if (i > 0) {
		if (kicker_shoot(i) != EXIT_SUCCESS) {

			return (PARSER_RET){CMD_ERROR, "Cannot Kick"};
		}
	}

	return (PARSER_RET){CMD_OK, 0x00};
}

PARSER_RET process_cmd_lext(char* buf, uint8_t len) {

	uint16_t i;

	if ((len < 1) || (len > 4)) {
		return (PARSER_RET){CMD_ERROR, "Invalid parameter"};
	}

	i = atoi(buf);
	if (i > 0) {
	
		uint8_t ret = kicker_lext(i);
	
		if (ret != EXIT_SUCCESS) {
			if(ret == EXIT_IGNORE) {
				return (PARSER_RET){CMD_IGNORE, 0x00};
			} else {
				return (PARSER_RET){CMD_ERROR, "Cannot extend left part"};	
			}
		}
	}

	return (PARSER_RET){CMD_OK, 0x00};
}

PARSER_RET process_cmd_rext(char* buf, uint8_t len) {

	uint16_t i;

	if ((len < 1) || (len > 4)) {
		return (PARSER_RET){CMD_ERROR, "Invalid parameter"};
	}

	i = atoi(buf);
	if (i > 0) {
	
		uint8_t ret = kicker_rext(i);
	
		if (ret != EXIT_SUCCESS) {
			if(ret == EXIT_IGNORE) {
				return (PARSER_RET){CMD_IGNORE, 0x00};
			} else {
				return (PARSER_RET){CMD_ERROR, "Cannot extend right part"};	
			}
		}
	}

	return (PARSER_RET){CMD_OK, 0x00};
}

PARSER_RET process_cmd_uext(char* buf, uint8_t len) {

	uint16_t i;

	if ((len < 1) || (len > 4)) {
		return (PARSER_RET){CMD_ERROR, "Invalid parameter"};
	}

	i = atoi(buf);
	if (i > 0) {

		uint8_t ret = kicker_uext(i);
	
		if (ret != EXIT_SUCCESS) {
			if(ret == EXIT_IGNORE) {
				return (PARSER_RET){CMD_IGNORE, 0x00};
			} else {
				return (PARSER_RET){CMD_ERROR, "Cannot extend upper part"};	
			}
		}
		
	}

	return (PARSER_RET){CMD_OK, 0x00};
}





#define STRING(a)	# a
#define XSTRING(s)	STRING(s)

const char* get_version(void) {
	return "SW Version: " XSTRING(MAJOR) "." XSTRING(MINOR) " Copyright 2007 CarpeNoctem Kai Baumgart <kb@zkar.de>";
}

