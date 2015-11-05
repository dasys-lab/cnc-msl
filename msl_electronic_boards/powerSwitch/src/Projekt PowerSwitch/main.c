/*
 * main.c
 *
 *  Created on: 27.04.2011
 *      Author: timoheumuller
 */

#define F_CPU 16000000UL

#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "port.h"
#include "global.h"

#define LED_GREEN		PB7
#define LED_AKT			PC0
#define LED_KICK		PB4
#define LED_RED			PB3

#define SW_MOT			PB1
#define SW_AKT			PB0
#define SW_KICK			PC7
#define SW_12V			???

#define SPEAKER			PD0

#define ADC_12V			0x08
#define ADC_EXT			0x09
#define ADC_24V			0x02

#define ADC_I_AKT		0x0A
#define ADC_I_KICK		0x05
#define ADC_I_MOT		0x06
#define ADC_I_12V		0x03
#define ADC_I_EXT		0x07

bool LED_GREEN = false;
bool LED_AKT = false;
bool LED_KICK = false;
bool LED_RED = false;

bool SW_MOT = false;
bool SW_AKT = false;
bool SW_KICK = false;
bool SW_12V = false;

bool SPEAKER = false;

uint16_t ADC_12V = 0;
uint16_t ADC_EXT = 0;
uint16_t ADC_24V = 0;

uint16_t ADC_I_AKT = 0;
uint16_t ADC_I_KICK = 0;
uint16_t ADC_I_MOT = 0;
uint16_t ADC_I_12V = 0;
uint16_t ADC_I_EXT = 0;


void initPorts() {
	DDRB = 0x00;
	DDRC = 0x00;
	DDRD = 0x01;
}

void getLEDS() {
	LED_GREEN = BIT_CHECK(PINB, LED_GREEN);
	LED_AKT = BIT_CHECK(PINC, LED_AKT);
	LED_KICK = BIT_CHECK(PINB, LED_KICK);
	LED_RED = BIT_CHECK(PINB, LED_RED);
}

void getSWS() {
	SW_MOT = BIT_CHECK(PINB, SW_MOT);
	SW_AKT = BIT_CHECK(PINB, SW_AKT);
	SW_KICK = BIT_CHECK(PINC, SW_KICK);
	SW_12V = BIT_CHECK(PIND, SW_12V);
}


void initADC() {
	ADMUX = 0;
	ADCSRA = 0;
	ADCSRB = 0;


	ADMUX = 0x40;	// Referenzspannung AVcc mit externem Kondensator an Aref
	ADCSRA = 0x8F;	// ADC Enabled, Interrupt Enabled, Prescaler: 128
	ADCSRB = 0x20;	// Aref Enabled, Free Running Mode
}

void startADC(uint8_t channel) {
	ADMUX = (ADMUX & 0xE0);
	ADMUX |= channel;

	ADCSRA |= (1<<ADSC);
}

uint16_t getADC(uint8_t channel) {
	uint16_t adc = ADCL;
	adc |= (ADCH << 8);

	return adc;
}

void startSpeaker() {
	SPEAKER = true;
}





#define POWER_ON_INT PD6
#define POWER_ON_EXT PD7
#define TASTER PC4
#define TASTER_LED	C,3

#define LED_GREEN_ON PORTD |= (1 << LED_GREEN);
#define LED_RED_ON PORTD |= (1 << LED_RED);
#define POWER_ON_EXT_ON PORTD |= (1 << POWER_ON_EXT);
#define POWER_ON_INT_ON PORTD |= (1 << POWER_ON_INT);

#define POWER_ON_INT_OFF PORTD &= ~(1 << POWER_ON_INT);
#define POWER_ON_EXT_OFF PORTD &= ~(1 << POWER_ON_EXT);
#define LED_GREEN_OFF PORTD &= ~(1 << LED_GREEN);
#define LED_RED_OFF PORTD &= ~(1 << LED_RED);

#define LED_GREEN_TOGGLE PORTD ^= (1 << LED_GREEN);
#define LED_RED_TOGGLE PORTD ^= (1 << LED_RED);

//w = w * 5000 / 1024;
//w = w * 147 / 47;

// 65,5 == 1V
// 1 == 0.015271775266 V
// 654 == 10 V
#define UINTERNCRITICAL 654
#define UEXTERNCRITICAL 654
// 720 == 11 V
#define UINTERNMIN 720
#define UEXTERNMIN 720

#define BUFFERLENGTH 256

#define LOUDNESS 8

	//w = w * 5000 / 1024;
	//w = w * 147 / 47;

uint8_t isTaster0() {
	static uint8_t count = 0;
	if (!(PINC & (1 << TASTER))) {
		++count;
	} else {
		count = 0;
	}
	if (count > 15) {
		count = 100;
		return 1;
	} else {
		return 0;
	}
}

void setFreq(uint16_t f) {
	ICR1 = f; // TOP-wert
	OCR1B = ICR1 / 2;

}

#define ADCBUFFERSIZE 10
#define ADCWMAX 950
volatile uint16_t uExternBuffer[ADCBUFFERSIZE];
volatile uint8_t uExternCounter;
volatile uint16_t uInternBuffer[ADCBUFFERSIZE];
volatile uint8_t uInternCounter;

void initADCBuffer(){
	for(int i=0; i<ADCBUFFERSIZE; i++){
		uExternBuffer[i] = 0;
		uInternBuffer[i] = 0;
	}
	uInternCounter = 0;
	uExternCounter = 0;
}

void initADCBufferExtern(){
	for(int i=0; i<ADCBUFFERSIZE; i++){
		uExternBuffer[i] = 0;
	}
	uExternCounter = 0;
}

void initADCBufferIntern(){
	for(int i=0; i<ADCBUFFERSIZE; i++){
		uInternBuffer[i] = 0;
	}
	uInternCounter = 0;
}

uint16_t getExtern()
{
	uint16_t sum = 0;
	for(int i=0; i<ADCBUFFERSIZE; i++)
	{
		sum += uExternBuffer[i];
	}
	return sum / ADCBUFFERSIZE;
	//return uExternBuffer[0];
}

uint16_t getIntern()
{
	uint16_t sum = 0;
	for(int i=0; i<ADCBUFFERSIZE; i++)
	{
		sum += uInternBuffer[i];
	}
	return sum / ADCBUFFERSIZE;
	//return uInternBuffer[0];
}

volatile uint8_t time;
uint8_t taster;
uint8_t beepTime;
uint8_t melody1[11] = { 10, 190, 180, 170, 160, 150, 140, 130, 120, 110, 100 };
uint8_t melody2[11] = { 10, 100, 110, 120, 130, 140, 150, 160, 170, 180, 190 };
uint8_t melody3[10] = { 4, 150, 150, 200, 200, 150, 150 };
uint8_t melody4[10] = { 4, 100, 150, 100, 150, 100, 150 };
uint8_t* melody = melody1;
uint8_t timeoutState4;
volatile uint8_t state = 0;

////////////
// states //
/////////////////////////////////////////////////////////////////////////////////
//   SOURCES	USING                                                          //
// 0 unknown 	none                                                           //
// 1 battery 	battery                                                        //
// 2 both	extern                                                         //
// 3 extern 	extern                                                         //
// 4 both	battery		wait for extern disconnect                     //
// 5 battery	battery		unexpected extern disconnect, acoustic warning //
// 6 battery	battery		low battery voltage                            //
// 7 battery	battery		critical battery voltage                       //
/////////////////////////////////////////////////////////////////////////////////


void playMelody(uint8_t* m) {
	melody = m;
	beepTime = melody[0] + 1;
}

void setVolume(uint16_t val) {
	uint16_t tmp = ICR1 / 100;
	OCR1B = tmp*val; // TOP-wert
}

// call 0.013056 sek
ISR(TIMER0_OVF_vect)
{
	volatile static uint8_t c = 0;
	++c;
	if (c == 8) { // 0.104448 sek
		c = 0;
		if (beepTime != 255) {
			--beepTime;
			if (beepTime == 0) {
				//setFreq(0);
				setVolume(0);
				beepTime = 255; // off
			} else {
				setFreq(melody[beepTime] * 100);
			}
		}

		if (timeoutState4 != 255) {
			--timeoutState4;
			if (timeoutState4 == 0) {
				timeoutState4 = 255;
				playMelody(melody3);
				state = 2;
			}
		}
		++time;
	}
	taster = isTaster0();
}

ISR(TIMER2_OVF_vect)
{

}

ISR(ADC_vect)
{
	static uint8_t channel = 0;
	static uint16_t lastExternVal = 0;
	static uint16_t lastInternVal = 0;
	
	// get channel (immer abwechselnd, bissel tricky, siehe datenblatt free running)
	if ((ADMUX & 7) == 0) 
	{ 
		lastExternVal = ADCW;
		if(lastExternVal < ADCWMAX)
		{
			uExternBuffer[uExternCounter] = lastExternVal;
			uExternCounter = (uExternCounter + 1) % ADCBUFFERSIZE;
			if (state == 4) // both; using battery; waiting for extern to disconnect
			{
				if (lastInternVal < UINTERNMIN) // battery lost
				{
					POWER_ON_INT_OFF
					POWER_ON_EXT_ON

					// reset buffer
					//
					initADCBufferIntern();

					state = 3;
				}
			}
		}
	} 
	else if ((ADMUX & 7) == 1)
	{
		lastInternVal = ADCW;
		uInternBuffer[uInternCounter] = lastInternVal;
		uInternCounter = (uInternCounter + 1) % ADCBUFFERSIZE;
		if (state == 2) // both; using extern
		{
			if (lastExternVal < UEXTERNMIN) // extern lost
			{
				POWER_ON_EXT_OFF
				POWER_ON_INT_ON
				
				// reset buffer
				//
				initADCBufferExtern();

				state = 5;
			}
		}
	}

	// change channel for next next conversion, next conversion already in process
	if (channel == 0) 
	{ 
		channel = 1;
	}
	else if (channel == 1)
	{
		channel = 0;
	}
	ADMUX = (1 << REFS0) | channel;
}

int main() {
	initADCBuffer();

	char buf[BUFFERLENGTH];

	DDRD = (1 << LED_RED) | (1 << POWER_ON_INT) | (1 << POWER_ON_EXT) | (1 << LED_GREEN);
	PORTD = 0;
	PORTD &= ~(1 << POWER_ON_EXT);
	PORTD &= ~(1 << POWER_ON_INT);

	//DDRC &= ~(1 << TASTER);
	//PORTC = 0;
	PORTC |= (1 << TASTER); // pull-up taster

	// init values
	beepTime = 255;
	timeoutState4 = 0;
	time = 0;

	// init pwm
	DDRB |= (1 << PB2);
	OCR1A = 50; // PWM einstellen, bevor der Timer startet
	ICR1 = 65535; // TOP-wert
	//OCR1B = ICR1 / 2;
	OCR1B = 0;

	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11); // 2-Kanal "non-inverting"
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); //Fastpwm, mit ICR1 als TOP

	//timer0
	TCCR0B = (1 << CS02) | (1 << CS00); // 1/1024
	TIMSK0 |= 1 << TOIE0;

	//timer2
	TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); // 1/1024
	TIMSK2 |= (1 << TOIE2);

	//uart
	uart_init(UART_BAUD_SELECT(9600, F_CPU));
	uart_puts("\n\nCN Power Switch V1.0\n");
	sei();

	ADMUX = (1 << REFS0);//aref = 5V (channel 0)
	//ADMUX = (1 << REFS0) | (1 << ADLAR);//aref = 5V (channel 0)
	
	ADCSRA = (1 << ADATE) | (1 << ADEN) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2) | (1 << ADIE) | (1 << ADSC); // free running, irq
	//ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADATE) | (1 << ADIE) | (1 << ADSC); // free running, irq
	ADCSRB = 0;
	//setFreq(0);
	setVolume(0);
	
	while (1) {

		uint16_t uIntern = getIntern();
		uint16_t uExtern = getExtern();

		sprintf(buf, "state: %01d extern %04d intern %04d\n", state, (int) uExtern, (int) uIntern);
		uart_puts(buf);
		if (state == 0) {
			if (uExtern > UEXTERNMIN) {
				state = 3;
			} else if (uIntern > UINTERNMIN) {
				state = 1;
			}
		} else if (state == 1) {
			LED_GREEN_ON
			LED_RED_OFF
			POWER_ON_EXT_OFF
			POWER_ON_INT_ON
			timeoutState4 = 255; //timer off

			if (uExtern > UEXTERNMIN) {
				state = 2;
			} else if (uIntern < UINTERNMIN) {
				state = 6;
			}
		} else if (state == 2) {
			LED_GREEN_OFF
			LED_RED_OFF
			POWER_ON_INT_OFF
			POWER_ON_EXT_ON

			if (taster && uIntern > UINTERNMIN) {
				state = 4;
				playMelody(melody4);
				timeoutState4 = 100;
			} else if (uIntern < UINTERNCRITICAL) {
				state = 3;
				playMelody(melody1);
			}
		} else if (state == 3) {
			LED_RED_OFF
			LED_GREEN_OFF
			POWER_ON_INT_OFF
			POWER_ON_EXT_ON

			if (uIntern > UINTERNMIN) {
				state = 2;
				playMelody(melody2);

			} else if (taster) {
				playMelody(melody3);
			}
		} else if (state == 4) {
			LED_GREEN_ON
			//blinken?
			LED_RED_OFF
			POWER_ON_EXT_OFF
			POWER_ON_INT_ON

			if (uExtern < UEXTERNCRITICAL) {
				state = 1;
			}
		} else if (state == 5) {
			//setFreq(10000);
			setVolume(LOUDNESS);
			LED_GREEN_ON
			LED_RED_ON
			//blinken
			POWER_ON_EXT_OFF
			POWER_ON_INT_ON

			if (taster) {
				state = 1;
				//setFreq(0);
				setVolume(0);
			} else if (uIntern < UINTERNMIN) {
				state = 6;
				setFreq(0);
			} else if (uExtern > UEXTERNMIN) {
				state = 2;
				//setFreq(0);
				setVolume(0);
			}
		} else if (state == 6) {
			LED_GREEN_ON
			LED_RED_ON
			POWER_ON_EXT_OFF
			POWER_ON_INT_ON
			//beep
			//setFreq(15000);
			setVolume(LOUDNESS);

			if (uIntern < UINTERNCRITICAL) {
				state = 7;
			} else if (uExtern > UEXTERNMIN) {
				state = 2;
				//setFreq(0);
				setVolume(0);
			}
		} else if (state == 7) {
			LED_GREEN_ON
			LED_RED_ON
			POWER_ON_EXT_OFF
			POWER_ON_INT_ON
			//beep
			//send halt
			//sleep 60
			//poweroff

		}

	}

	//	sprintf(buf, "measure: %04d \n", (int) b2);
	//	uart_puts(buf);
	return 0;
}

ISR(TIMER1_OVF_vect) {
	static uint8_t i = 0;

	if (i >= 2){
		SPEAKER = false;
	} else {i++;}
}

