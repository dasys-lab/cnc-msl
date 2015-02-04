#define F_CPU 16000000UL

#include "global.h"
#include "defaults.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#include "uart.h"
#include "util.h"
#include "port.h"
#include "spi.h"
#include "adns3080.h"

#define BUFFERLENGTH 256

// Some Buffers
uint8_t img[1536];
int8_t motion[2];
int8_t motionBurst[7];

int main(void)
{
	uart1_init(UART_BAUD_SELECT(9600, F_CPU));
	sei();	// enable interrupts

	spi_init();
		
	//uart1_puts("Init optical flow...");
	
	reset();
	_delay_us(35000); // wait t_PU-RESET
	setConfigurationBits(0x00); // set resolution (0x00 = 400 counts per inch, 0x10 = 1600 cpi)
	
	while(1)
	{	
		
		//		getFrameBurst(img);
// 		printFrame(img);
		
		_delay_us(30000);				
 		getMotionBurst(motionBurst);
		
 		printMotionBurst(motionBurst);
// 		printMotionBurstInRobot(motionBurst);
//		_delay_us(50000);		// für MATLAB read_ball_motion() _delay_us(50000)		
	}
	
	return 0;
}
