#include <avr/interrupt.h>
#include <avr/io.h>
#include "defaults.h"
#include "global.h"
#include "messages.h"

void     pwm_init(void) {
	
	//set port direction
	//servo
	SET_OUTPUT(SERVO_PWM);
	//motors
	SET_OUTPUT(MOTOR_LEFT_PWM);
	SET_OUTPUT(MOTOR_RIGHT_PWM);
	SET_OUTPUT(MOTOR_STOP_PWM);

	//set pwm waveform, prescales, enable
  	TCCR1A = (1<<COM1A1) | (1<<WGM11);
  	TCCR3A = (1<<COM3A1) | (1<<COM3B1) | (1<<COM3C1) | (1<<WGM31);
  	TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS10) | (1<<CS11);
  	TCCR3B = (1<<WGM33) | (1<<WGM32) | (1<<CS30);

	//set frequeny to 50hz?
	ICR1 = 5000;
	//set frequeny to 35khz
	ICR3 = 450;

	OCR1A = 250;
	OCR3A = 0;
	OCR3B = 0;
	OCR3C = 0;
}

void     pwm_set(uint8_t pwm, uint16_t val) {

	if( pwm == 0 ) {
		//min:237 max:512
		if( val < 237 ) {
			val = 237;
			debug("here");
		}
		if( val > 512 ) {
			val = 512;
		}
		OCR1A = val;
	} else if( pwm == 1) {
		OCR3A = val;
	} else if( pwm == 2 ) {
		OCR3B = val;
	} else if( pwm == 3 ) {
		OCR3C = val;
	}
}
