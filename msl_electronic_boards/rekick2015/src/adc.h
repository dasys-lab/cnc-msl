/*
 * adc.h
 *
 *  Created on: Sep 9, 2016
 *      Author: cn
 */

#ifndef CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_ADC_H_
#define CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_ADC_H_

#include <avr/io.h>

void adc_init();
int8_t adc_start_conversion(int8_t muxMode);
void adc_handler();
int16_t adc_read(int8_t muxMode);
//ISR(ADC_vect);


#endif /* CNC_MSL_MSL_ELECTRONIC_BOARDS_REKICK2015_SRC_ADC_H_ */
