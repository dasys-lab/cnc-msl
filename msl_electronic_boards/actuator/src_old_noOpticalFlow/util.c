/*
 * util.c
 *
 *  Created on: 30.11.2010
 *      Author: philipp
 */
#include <avr/io.h>
#include "util.h"
#include "uart.h"
#include <stdio.h>

char buf[BUFFERLENGTH];

void printRevision(unsigned int value)
{
	sprintf(buf, "sensor revision: %u\n\n", value);
	uart1_puts(buf);
}

void printValue(unsigned int value)
{
	sprintf(buf, "%u\n", value);
	uart1_puts(buf);
}
