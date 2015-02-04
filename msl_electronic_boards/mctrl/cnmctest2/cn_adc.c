#include "cn_adc.h"
#include "cn_motor.h"
#include "adc.h"

uword cn_adc_index;
cn_adc_res cn_adc_results[CN_ADC_CHAN_COUNT];

ubyte cn_adc_channels[] = { CN_ADC_CHAN_SUPPLY,
							CN_ADC_CHAN_MOTOR1,
							CN_ADC_CHAN_MOTOR2,
							CN_ADC_CHAN_MOTOR3 };
							
ubyte cn_adc_motors[]   = { 0xFF,
							MOTOR1,
							MOTOR2,
							MOTOR3 };

void cn_adc_init() {
	uword i;
	
	for(i = 0; i < CN_ADC_CHAN_COUNT; ++i) {
		cn_adc_results[i].voltage = 0;
		cn_adc_results[i].pwm = 0;
	}
	
}

void cn_adc_isr() {

	ubyte pwm_motor;
	
	// determine motor
	pwm_motor = cn_adc_motors[cn_adc_index];
	
	// save values
	cn_adc_results[cn_adc_index].voltage = (ADDAT & 0x03FF);
	
	if(pwm_motor != 0xFF) {
		cn_adc_results[cn_adc_index].pwm = getMotorPWM(pwm_motor);
	}
	
	// prepare next channel
	++cn_adc_index;

    if(cn_adc_index >= CN_ADC_CHAN_COUNT) return; // all channels done
	
	ADC_SetConvMode(ADC_FIXED, cn_adc_channels[cn_adc_index]); // set new channel
	
	ADC_StartConv(); // trigger next channel

}

void cn_adc_trigger_conversions() {
	
	// set first channel
	cn_adc_index = 0;
	ADC_SetConvMode(ADC_FIXED, cn_adc_channels[cn_adc_index]);
	
	// trigger ADC conversion
	ADC_StartConv();
	
	// result is read in ISR
}

uword getSupplyVoltage() {

	uword i;
	uword voltage;
	
	for(i = 0; i < CN_ADC_CHAN_COUNT; ++i) {
		if(cn_adc_channels[i] == CN_ADC_CHAN_SUPPLY) break;
	}
	
	// TODO: better error signaling
	if(cn_adc_channels[i] != CN_ADC_CHAN_SUPPLY) return 0xFFFF;

#ifdef TMC
	// original tmc
	//voltage = cn_adc_results[i].voltage * 40 / 1023;
	voltage = (uword) (cn_adc_results[i].voltage * 4000l / 1023l);
#endif
#ifdef VMC
	//voltage = (uword) ((cn_adc_results[i].voltage *5*10000) / 1023 / 10);
	voltage = (uword) ((cn_adc_results[i].voltage * 55314l) / 10000l);
#endif
	
	return voltage;
	
}

uword getMotorVoltage(ubyte motor) {

	uword i;

	// find index for results
	// TODO: maybe do this in adc_init for all
	for(i = 0; i < CN_ADC_CHAN_COUNT; ++i) {
		if(cn_adc_motors[i] == motor) break;
	}
  
	// TODO better error signaling
	if(cn_adc_motors[i] != motor) return 0xFFFF;
	
 	return cn_adc_results[i].voltage;
	
}

