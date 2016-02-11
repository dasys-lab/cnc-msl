#ifndef	DEFAULTS_H
#define	DEFAULTS_H

/**
 * @file defaults.h
 * @brief DefaultConfig
 * here the pins for the relaispins, SPI, status and error leds and some names are defined */




#define LOGGING   1

// reagieren auf werte -- ueber jumper
#define DONT_REAC 500
#define NO_RELAIS 111
#define HIGHER	1
#define LOWER	0

/**@brief the clock adjust pin controlled by the poti */
#define CLOCK_REGULATOR	6

//#define	MCP2515_INT		B,2

#define isON PB0 // On schalter

/**@brief the green led shows the sending activity */
#define showON PC0 // gruene led

/**@brief the green led near to the red led shows incoming commands */
#define showCommand PC5
/**@brief this led shows errors with command processsing */
#define errorLed PC7


#define lastPin1 PC1 // schaltet lasten mit transistor und relais
#define lastPin2 PC2 // schaltet lasten mit transistor und relais
#define lastPin3 PC3 // schaltet lasten mit transistor und relais


// SPI ------------------

#define	P_MOSI PB5
#define	P_MISO PB6
#define	P_SCK PB7

#define	P_CS PB4


//----------can--------------
#define ETH2CAN_ID		0x00
#define MOTOR_ID		0x20
#define COMPASS_ID		0x40
#define REKICK_ID		0x60
#define SERVO_ID        0x80
#define M_BOARD_ID		0x50
#define M_BOARD_ID2		0x55

#define PRIORITY_HIGH	0x20
#define PRIORITY_NORM	0x40
#define PRIORITY_LOW	0x80



#endif	// DEFAULTS_H