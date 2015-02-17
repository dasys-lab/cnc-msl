#define F_CPU 1000000UL 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart.h"


int main() {
    unsigned char i;
    char c='A';
    int led;
 
    //DDRD = 0xff;          // PB0 an PORTB als Ausgang setzen
    DDRD = (1 << PD4) | (1<<PD3);
    PORTD ^= (1 << PD4);
    uart_init(UART_BAUD_SELECT(2400, F_CPU));
    sei();
    //uart_puts("AAATEST");
    //uart_putc(c);
    while(1)                  // Für immer
    {
	c = uart_getc();
	if (c & UART_NO_DATA) {
	  PORTD ^= (1 << PD3);
          continue;
        }

        // check for error
        if (c & UART_FRAME_ERROR) {
          continue;
        }
        if (c & UART_OVERRUN_ERROR) {
          continue;
        }
        if (c & UART_BUFFER_OVERFLOW) {
          continue;
        }
	if(c==0x00) continue;
	PORTD ^= (1 << PD4);
	uart_putc(c+1);
/*		for(led=2; led<5; led++)		{
        		PORTD = (1 << led);  // Toggle PB0 z.B. angeschlossene LED
 

	        	for (i=1; i<=10; i++)         
        		    	_delay_ms(10);    
		}
		for(led=3; led>2; led--continue
	        	PORTD = (1 << led);  // Toggle PB0 z.B. angeschlossene LED
 

        		for (i=1; i<=10; i++)         
		            	_delay_ms(10);    
		}*/
    }
            
	
	return 0;
}
