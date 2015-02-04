#include <avr/io.h>
#include "defaults.h"
#include "global.h"

// use AVCC as reference
#define ADCREF  (1 << REFS0)

void toggle_status_led(void) {
	
	PORTD ^= (1 << PD6);
}

uint16_t read_adc(void) {

	uint8_t i;
	uint16_t res = 0;

	// wait for end of last (test-) conversion
	while (ADCSRA & (1 << ADSC));

	// read channel
	for (i = 0; i < 3; i++) {
		ADCSRA |= (1 << ADSC);
		while (ADCSRA & (1 << ADSC));
		res += ADCW;
	}
	res /= 3;

	return res;
}

void select_adc_channel(uint8_t mux) {

	uint8_t last_channel, new_channel;

	last_channel = ADMUX;
	new_channel = ADCREF | mux;

	// change channel
	if (last_channel != new_channel) {
		ADMUX = new_channel;
		// start a test-conversion, but do not wait
		ADCSRA |= (1 << ADSC);
	}

	return;
}

void ports_init(void) {

	//init left motor
	SET_OUTPUT(MOTOR_LEFT_DIR);
	SET_OUTPUT(MOTOR_LEFT_VEL);
	SET_OUTPUT(MOTOR_LEFT_RESET);
	SET_INPUT(MOTOR_LEFT_ERROR_1);
	SET_INPUT(MOTOR_LEFT_ERROR_2);
	PORTC |= ( 1<<PC3 );    /* internen Pull-Up an PC7 aktivieren */
	PORTC |= ( 1<<PC4 );    /* internen Pull-Up an PC7 aktivieren */

	//init right motor
	SET_OUTPUT(MOTOR_RIGHT_DIR);
	SET_OUTPUT(MOTOR_RIGHT_VEL);
	SET_OUTPUT(MOTOR_RIGHT_RESET);
	SET_INPUT(MOTOR_RIGHT_ERROR_1);
	SET_INPUT(MOTOR_RIGHT_ERROR_2);
	PORTC |= ( 1<<PD3 );    /* internen Pull-Up an PC7 aktivieren */
	PORTC |= ( 1<<PD4 );    /* internen Pull-Up an PC7 aktivieren */

	//init right motor
	SET_OUTPUT(MOTOR_STOP_DIR);
	SET_OUTPUT(MOTOR_STOP_VEL);
	SET_OUTPUT(MOTOR_STOP_RESET);
	SET_INPUT(MOTOR_STOP_ERROR_1);
	SET_INPUT(MOTOR_STOP_ERROR_2);
	SET_INPUT(MOTO_STOP_POTI);
	PORTC |= ( 1<<PE3 );    /* internen Pull-Up an PC7 aktivieren */
	PORTC |= ( 1<<PE4 );    /* internen Pull-Up an PC7 aktivieren */
	
	//TODO set adc for poti
	// select adc multiplexer channel
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	ADMUX = ADCREF;
	select_adc_channel(6);

}

uint16_t get_capacitors_voltage(void) {
	
	select_adc_channel(6);
	uint16_t ret = read_adc();
	uint16_t val = ret *0.41;


	if( val < 50 )
	{
		OCR1B = 10;
		OCR1A = 104;
	}
	else if( val < 70 )
	{
		OCR1B = 20;
		OCR1A = 104;
	}
	else if( val > 70 && val < 90 )
	{
		OCR1B = 52;
		OCR1A = 104;
	}
	else if( val > 90 && val < 120 )
	{
		OCR1B = 52;
		OCR1A = 100;
	}
	else if( val > 120 && val < 160 )
	{
		OCR1B = 52;
		OCR1A = 96;
	}
	else if( val > 160 && val < 200 )
	{
		OCR1B = 52;
		OCR1A = 92;
	}
	else if( val > 200 && val < 240 )
	{
		OCR1B = 52;
		OCR1A = 90;
	}
	else if( val > 240 )
	{
		OCR1B = 52;
		OCR1A = 88;
	}

	return val;
}

float get_supply_voltage(void) {
	
	select_adc_channel(1);
	uint16_t ret = read_adc();

	return ret * 0.0394629;
}

uint16_t get_supply_raw_voltage(void) {
	
	select_adc_channel(1);
	uint16_t ret = read_adc();

	return ret;
}

