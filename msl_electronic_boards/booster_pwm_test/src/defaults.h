#ifndef	DEFAULTS_H
#define	DEFAULTS_H

#define	P_MOSI			B,5
#define	P_MISO			B,6
#define	P_SCK			B,7

#define	MCP2515_CS		B,4
#define	MCP2515_INT		B,2

#define POWER			A,4
#define RELEASE			A,3
#define ROTOR_CHECKLOCK	A,2
#define ROTOR_INTERLOCK	A,5
#define NOT_DISCHARGE	B,0
#define LED_RED1		C,7
#define LED_GREEN1		C,6
//#define LED_RED2		D,1 // disabled, we use rx/tx for the UART
//#define LED_GREEN2	D,0 // disabled, we use rx/tx for the UART
#define PWM_BOOST		D,4
#define PWM_SERVO1		D,7
#define PWM_SERVO2		B,3

#endif	// DEFAULTS_H
