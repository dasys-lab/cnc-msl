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
void spi_init(void)
{
	/*
	Set directions for the SPI lines of the PORTB
	Configure PB2=output (NSS)
	Configure PB3=output (MOSI)
	Configure PB5=output (SCK) */

	SET_OUTPUT(P_MOSI);
	SET_OUTPUT(P_SCLK);
	SET_OUTPUT(P_NCS);

	SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0) | (1 << SPR1);

	SET(P_NCS);
}

uint8_t spi_exch (uint8_t output)
{
	SPDR = output;
	/* Start transmission */
	/* Wait till a transmission and reception are completed */
	while(!(SPSR & (1<<SPIF)));

	/* Return Data Register */
	return SPDR;
}
