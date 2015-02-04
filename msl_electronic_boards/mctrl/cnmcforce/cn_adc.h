#ifndef CN_ADC
#define CN_ADC

#include "main.h"

#define CN_ADC_CHAN_COUNT 4

#ifdef TMC
	#define CN_ADC_CHAN_MOTOR1 ADC_ANA_0
	#define CN_ADC_CHAN_MOTOR2 ADC_ANA_2
	#define CN_ADC_CHAN_MOTOR3 ADC_ANA_1
	#define CN_ADC_CHAN_SUPPLY ADC_ANA_3
#endif
#ifdef VMC
	#define CN_ADC_CHAN_MOTOR1 ADC_ANA_9
	#define CN_ADC_CHAN_MOTOR2 ADC_ANA_8
	#define CN_ADC_CHAN_MOTOR3 ADC_ANA_5
	#define CN_ADC_CHAN_SUPPLY ADC_ANA_0
#endif

#define CN_ADC_STRUCT_SUPPLY 0
#define CN_ADC_STRUCT_MOTOR1 1
#define CN_ADC_STRUCT_MOTOR2 2
#define CN_ADC_STRUCT_MOTOR3 3

typedef struct {
	uword voltage;
	uword pwm;
} cn_adc_res;

void cn_adc_init();
void cn_adc_isr();
void cn_adc_trigger_conversions();
uword getSupplyVoltage();
uword getMotorVoltage(ubyte motor);


extern cn_adc_res cn_adc_results[];
extern ubyte cn_adc_channels[];
extern ubyte cn_adc_motors[];
extern uword cn_adc_index;

#endif