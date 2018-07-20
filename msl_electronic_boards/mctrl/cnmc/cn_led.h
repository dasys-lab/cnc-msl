#ifndef CN_LED
#define CN_LED

#include "main.h"

#ifdef TMC
	#define GREEN_LED_PIN    P3_P3_9
#endif
#ifdef VMC
	#define GREEN_LED_PIN    P4_P4_7
	#define GREEN_LED_DIR    DP4_DP4_7
	#define RED_LED_PIN      P4_P4_4
	#define RED_LED_DIR      DP4_DP4_4
#endif

void init_led(void);
void greenLedOn(void);
void greenLedOff(void);
void greenLedToggle(void);

#ifdef VMC
void redLedOn(void);
void redLedOff(void);
void redLedToggle(void);
#endif

#endif