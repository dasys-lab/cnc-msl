/*
 * spi_test.c
 *
 *  Created on: 14.01.2013
 *      Source: http://www.edaboard.com/thread150628.html
 */

#include <avr/io.h>
#include "spi.h"


void InitSPI(void)
{
	DDRB = (1<<PB4)|(1<<PB5) | (1<<PB7);	 // Set MOSI , SCK , and SS output
	SPCR = ( (1<<SPE)|(1<<MSTR) | (1<<SPR1) |(1<<SPR0));	// Enable SPI, Master, set clock rate fck/128  
}

void WriteByteSPI(unsigned char byte)
{
	
	SPDR = byte;					//Load byte to Data register
	while(!(SPSR & (1<<SPIF))); 	// Wait for transmission complete 
	
}

char ReadByteSPI(char addr)
{
	SPDR = addr;					//Load byte to Data register
	while(!(SPSR & (1<<SPIF))); 	// Wait for transmission complete 
	addr=SPDR;
	return addr;
}