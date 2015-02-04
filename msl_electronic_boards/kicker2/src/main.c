/*
 * $Id: main.c 3156 2008-03-12 15:09:28Z cn $
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
/*
 * note:
 *
 *  the kicker number one is on the opposite side of axis number one
 *  the kicker number two is on the opposite side of axis number two
 */

#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "uart.h"
#include "ports.h"
#include "kicker.h"
#include "parser.h"

#define UART_BAUD_RATE			57600


int main(void) {

	// init
	uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));

	// enable interrupts after resetting the kicker
	sei();
	uart_puts_P("* Interrupts enabled. - ");
	
	ports_init();
	uart_puts_P("Ports initialized. - ");
	
	kicker_init();
	uart_puts_P("Kicker initialized. - ");

	//kicker_uext(500);

	uart_puts_P("Device initialized. Enter main loop\n");

	for (;;) {
		parser_process_uart();
	}

	return 0;
}

