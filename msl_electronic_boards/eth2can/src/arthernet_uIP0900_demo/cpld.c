/*
 * Routines to initialise the CPLD's bank-switch register and SPI_CS-port.
 *
 * 2005 by Patrick 'OldBug' Dohmen
 */

#include "cpld.h"

volatile unsigned char *pSPI_CS = (unsigned char *) 0x1200;
volatile unsigned char *pBank_Reg = (unsigned char *) 0x1100;

void cpld_init(void) __attribute__ ((naked)) __attribute__ ((section (".init3")));

void
cpld_init(void)
{
	*pBank_Reg = (1 << B1P1);		/* setup bank-switching in CPLD	*/
	*pSPI_CS = 0xFF;			/* default value of CS-Port	*/
}
