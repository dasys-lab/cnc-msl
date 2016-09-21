/*
 * adc.c
 *
 *  Created on: Sep 9, 2016
 *      Author: cn
 */


#include "booster.h"
#include "defaults.h"
#include "global.h"

#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint16_t adc_value;
volatile int8_t adc_muxMode;
volatile bool adc_ready;


void adc_init()
{
	ADMUX = 0x00;
	ADMUX |= (1 << REFS0);	// AVcc with external capacitor on AREF pin

	ADCSRA = 0x00;
	ADCSRA |= (1 << ADEN) | (1 << ADIE);		// Enable ADC, Enable Interupt
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);	// Set Prescaler to 128 (16MHz / 128 = 125kHz)

	ADCSRB = 0x00;
	ADCSRB |= (1 << AREFEN);
}

int8_t adc_start_conversion(int8_t muxMode)
{
	// Check if there is a conversion running
	if (ADCSRA & (1 << ADSC)) {
		// Old Conversion running
		return -1;
	}

	ADMUX = ADMUX & 0xC0;		// Save VRef selection Bits
	ADMUX |= muxMode & 0x1F;	// Choose muxMode

	ADCSRA |= (1 << ADSC);		// Start Conversion
	adc_ready = false;

	return 1;
}

void adc_handler()
{
	static uint8_t mux = 0;

	// Check if there is a conversion running
	if (ADCSRA & (1 << ADSC)) {
		return;
	}

	switch(mux)
	{
		case ADC_24V_LOGIC:
			mux = ADC_24V_BOOSTER;
			break;

		case ADC_24V_BOOSTER:
			mux = ADC_CAP;
			break;

		case ADC_CAP:
			mux = ADC_24V_LOGIC;
			break;

		default:
			mux = ADC_CAP;
			break;
	}

	adc_start_conversion(mux);
}

int16_t adc_read(int8_t muxMode)
{
	// Check if there is a conversion running
	if (ADCSRA & (1 << ADSC))
	{
		// Old Conversion running
		return -1;
	}

	uint8_t reg = ADMUX & 0xC0;		// Save VRef selection Bits
	reg |= muxMode & 0x1F;			// Choose muxMode

	ADCSRA |= (1 << ADSC);			// Start Conversion

	while (ADCSRA & (1 << ADSC)) {

	}

	int16_t adc = (ADCH << 8) | ADCL;
	return adc;
}

ISR(ADC_vect)
{
	cli();
	adc_muxMode = ADMUX & 0x1F;
	adc_ready = true;

	switch(adc_muxMode)
	{
		case ADC_24V_LOGIC:
			adc_logic_raw = (ADCH << 8) | ADCL;
			break;

		case ADC_24V_BOOSTER:
			adc_booster_raw = (ADCH << 8) | ADCL;
			break;

		case ADC_CAP:
			adc_capacitor_raw = (ADCH << 8) | ADCL;
			break;

		default:
			adc_value = (ADCH << 8) | ADCL;
			break;
	}

	sei();
}
