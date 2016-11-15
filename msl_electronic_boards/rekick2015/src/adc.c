/*
 * adc.c
 *
 *  Created on: Sep 9, 2016
 *      Author: cn
 */

#include "adc.h"

#include "defaults.h"
#include "global.h"

#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint16_t adc_value;
volatile int8_t adc_muxMode;
volatile bool adc_ready;

enum adcMUX {
	ADC_24V_LOGIC = 1,
	ADC_24V_BOOSTER,
	ADC_CAP,
};


void adc_init()
{
	ADMUX = 0x00;
	ADMUX |= (1 << REFS0);	// AVcc with external capacitor on AREF pin

	ADCSRA = 0x00;
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);	// Set Prescaler to 128 (16MHz / 128 = 125kHz)
	ADCSRA |= (1 << ADEN) | (1 << ADIE);		// Enable ADC, Enable Interupt

	ADCSRB = 0x00;
	//ADCSRB |= (1 << AREFEN);

	adc_start_conversion(ADC_CAP);
}

int8_t adc_start_conversion(int8_t channel)
{
	// Check if there is a conversion running
	if (ADCSRA & (1 << ADSC)) {
		// Old Conversion running
		return -1;
	}

	ADMUX = (ADMUX & 0xC0) | (channel & 0x0F);

	// Start Conversion
	ADCSRA |= (1 << ADSC);

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
	reg |= muxMode & 0x0F;			// Choose muxMode

	ADCSRA |= (1 << ADSC);			// Start Conversion

	while (ADCSRA & (1 << ADSC)) {

	}

	int16_t adc = (ADCH << 8) | ADCL;
	return adc;
}

ISR(ADC_vect)
{
	adc_muxMode = ADMUX & 0x0F;
	adc_value =  ADCL | (ADCH << 8);

	switch(adc_muxMode)
	{
		case ADC_24V_LOGIC:
			booster_setLogicRawVoltage(adc_value);
			adc_start_conversion(ADC_24V_BOOSTER);
			break;

		case ADC_24V_BOOSTER:
			booster_setBoosterRawVoltage(adc_value);
			adc_start_conversion(ADC_CAP);
			break;

		case ADC_CAP:
			booster_setCapacitorRawVoltage(adc_value);
			adc_start_conversion(ADC_24V_LOGIC);
			break;

		default:
			adc_start_conversion(ADC_24V_LOGIC);
			break;
	}
}
