#include <avr/io.h>

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

	// select adc multiplexer channel
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	ADMUX = ADCREF;
	select_adc_channel(6);

	// Set green status led on 
	DDRD |= (1 << DDD6); 

}

uint16_t get_capacitors_voltage(void) {
	
	select_adc_channel(6);
	uint16_t ret = read_adc();

	return ret * 0.2906;
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

