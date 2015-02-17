#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <string.h>
#include <stdio.h>
#include <avr/io.h>
//#include "pwm.c"

//************CONFIGURE PORTS************
//configure ports for input or output - specific to ATmega168
void configure_ports(void)
{
	DDRC = 0xFF;  //configure all C ports for input.  0xFF is  '0b11111111'
	PORTC = 0x00; //make sure pull-up resistors are turned off
	DDRD = 0xFF;  //configure all D ports for output.  0xFF is  '0b11111111'
	DDRB = 0xCF;  //configure B ports 0, 1, 2,3, 6, 7 for output.  0xCF is  '0b11001111
}

int main(void) {
	
	configure_ports();

	// Start PWM
	// Phase Correct, TOP is OCR1A, Update OCR1x at TOP, prescaler 8
	// Clear OC1B on Compare Match when upcounting. Set OC1B on Compare
	// Match when downcounting.
	TCCR1A = (1 << COM1B1) | (1 << WGM11) | (1 << WGM10);
	TCCR1B = (1 << WGM13) | (1 << CS10);
	OCR1A = 130;								// set period to 20 ms
	OCR1B = 72;


	TCCR0A = _BV(WGM01);
 	TCCR0B = _BV(CS00) | _BV(CS02);
 	OCR0A = 17; 
 	TIMSK0 = _BV(OCIE0A);

//	pwmInit910(); // Intialize PWM for timer which controls pins 9 and 10 ( since pins 9 and 10 are on the same timer)
//    	pwmOn9(); // turn on PWM for pin 9
//    	pwmInit56(); //Initialize timer for pins 5 and 6(since pins 5 and 6 are on the same timer)
//    	pwmOn5(); // turn on PWM for pin 5
//      	pwmSet5(255); //pin6   which is 8 bit PWM
//    	pwmSet9(65535); //pin9 which is 16 bit PWM
//	pwmSet10(255);

	volatile int test = 0;
	while(1) {
		test+=1;
		
	}
	
	return 0;
}
