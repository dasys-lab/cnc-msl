/*
 * spi.c
 *
 *  Created on: 21.01.2011
 *      Author: philipp
 */

#include <avr/io.h>
#include "spi.h"

/* The example code assumes that the part specific header file is included */


// SPI_Init
void SPI_Init(void)
{
	/*
	Set directions for the SPI lines of the PORTB
	Configure PB2=output (NSS)
	Configure PB3=output (MOSI)
	Configure PB5=output (SCK) */

	DDRB |= (1<<PB0) | (1<<PB1) | (1<<PB2);
	DDRA |= (1<<PA6);

	SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0) | (1 << SPR1);

	NSS_High();
}

unsigned int SPI_EXCH (unsigned char output)
{
	SPDR = output;
	/* Start transmission */
	/* Wait till a transmission and reception are completed */
	while(!(SPSR & (1<<SPIF)));

	/* Return Data Register */
	return SPDR;
}
