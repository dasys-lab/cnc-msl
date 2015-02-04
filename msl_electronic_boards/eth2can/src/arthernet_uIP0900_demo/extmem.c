/*
 * Routines to configure and enable the external SRAM interface.
 *
 * 2005 by Patrick 'OldBug' Dohmen
 */

#include <avr/io.h>

#include "cpld.h"

/*
 * Bank_0 is reserved for dynamic memory allocation; malloc().
 * Setup a pointer to the external SRAM where the CPLD's Bank_Reg Bank_1 nibble
 * is configured to point to.
 */
volatile unsigned char *pBank_1 = (unsigned char *) 0x8000;

/*
 * Initialise the external SRAM-interface early.
 * See avr-libc FAQ
 */
void extmem_init(void) __attribute__ ((naked)) __attribute__ ((section (".init1")));

void
extmem_init(void)
{
#if defined(__ARTHERNET__)
	MCUCR = (1 << SRE);
#else
#	error "WRONG PLATFORM!"
#endif
}
