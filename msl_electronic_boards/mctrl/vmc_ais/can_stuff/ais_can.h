/** \file
 // * $Id: $       
 */
#ifndef  __CAN_STUFF__
#define  __CAN_STUFF__

#include "system/ais_select.h"

extern unsigned char cansend_data[8];

 


#include <reg167.h>


/**
 * Type definition for a volatile byte.
 */
typedef volatile unsigned char can_byte;

/**
 * Type definition for a volatile word.
 */
typedef volatile unsigned int can_word;

/**
 * Type definition for CAN register.
 */
typedef union {
  can_word word;        /**< Whole 16-bit register. */
  can_byte byte[2];     /**< Two parts of 8-bit each. */
} canreg;

/**
 * Type definition for CAN message object.
 */
typedef struct {
  canreg    C1MCR;      /**< Message Control Register. */
  canreg    C1UAR;      /**< Upper Arbitration Register. */
  canreg    C1LAR;      /**< Lower Arbitration Register. */
  can_byte  C1MCFG;     /**< Message Configuration Register. */
  can_byte  C1DA0;      /**< Data Area. */
  can_byte  C1DA1;      /**< Data Area. */
  can_byte  C1DA2;      /**< Data Area. */
  can_byte  C1DA3;      /**< Data Area. */
  can_byte  C1DA4;      /**< Data Area. */
  can_byte  C1DA5;      /**< Data Area. */
  can_byte  C1DA6;      /**< Data Area. */
  can_byte  C1DA7;      /**< Data Area. */
  can_byte  C1RESVD;    /**< Reserved. */
} messageobject;

#define BUSY 0
#define FREE -1
#define OCCUPIED 0
#define DONE -1

#define CANBUFLEN 16


struct Can_Receive {

			unsigned char finished;		// -1 = done
			unsigned char occupied;		// 0 = empty
			messageobject can_buf;
			};

unsigned char can_receive(void);
int can_send(char mobj);

extern short can_index;
extern short can_index_last;
extern short can_done;
extern struct Can_Receive  can_rec_buf[CANBUFLEN];
extern short can_answer(void) ;
short copy2nextBuf( messageobject* Received);
short nextCANindex(void);

extern unsigned char cansend_data[8];

// ---------------------------------
// Accept Adr
// ---------------------------------
#define ACCEPT						0x7f0
#define	COMMAND_MASK				0x07f

#define CAN_BaseAdr_TX				0x780 
#define CAN_BaseAdr_RX				0x700 

// ---------------------------------
// System Commands
//
#define _CMDGRP_SYSTEM_BASIC_IN_		0x10
#define _CMDGRP_SYSTEM_BASIC_OUT_		0x11
#define _CMDGRP_SYSTEM_MAINTANANCE_IN_	0x1A
#define _CMDGRP_SYSTEM_MAINTANANCE_OUT_	0x1B


// ---------------------------------
// Motor Control Groups
#define _CMDGRP_MOTOR_CONFIN_		0x50
#define _CMDGRP_MOTOR_CONFOUT_		0x51
#define _CMDGRP_MOTOR_CTRL_			0x52

#define _CMDGRP_MOTOR_STATUSIN_		0x54
#define _CMDGRP_MOTOR_STATUSOUT_	0x55
#define _CMDGRP_CONTROLLER_PARIN_	0x56
#define _CMDGRP_CONTROLLER_PAROUT_	0x57

#define _CMDGRP_MOTOR_ERR_			0x59
#define _CMDGRP_MOTOR_MASK_			0x50
// ---------------------------------

/** \defgroup CANIds CAN identifiers used by KURT2's firmware
 */
/*@{*/
#define CAN_CMDGRP_SYSTEM_BASIC_IN_			CAN_BaseAdr_RX | (_CMDGRP_SYSTEM_BASIC_IN_ & COMMAND_MASK ) 		// RX
#define CAN_CMDGRP_SYSTEM_BASIC_OUT_			CAN_BaseAdr_TX | (_CMDGRP_SYSTEM_BASIC_OUT_ & COMMAND_MASK ) 		// TX
#define CAN_CMDGRP_SYSTEM_MAINTANANCE_IN_	CAN_BaseAdr_RX | (_CMDGRP_SYSTEM_MAINTANANCE_IN_ & COMMAND_MASK ) 		// RX
#define CAN_CMDGRP_SYSTEM_MAINTANANCE_OUT_		CAN_BaseAdr_TX | (_CMDGRP_SYSTEM_MAINTANANCE_OUT_ & COMMAND_MASK ) 		// TX

#define CAN_CMDGRP_MOTOR_CONFIN_   		CAN_BaseAdr_RX | (_CMDGRP_MOTOR_CONFIN_ & COMMAND_MASK ) 		// RX
#define CAN_CMDGRP_MOTOR_CONFOUT_ 			CAN_BaseAdr_TX | (_CMDGRP_MOTOR_CONFOUT_ & COMMAND_MASK )		    // TX
#define CAN_CMDGRP_MOTOR_CTRL_			CAN_BaseAdr_RX | (_CMDGRP_MOTOR_CTRL_ & COMMAND_MASK ) 		// RX
#define CAN_dummy      					0xffff							// NOT used
#define CAN_CMDGRP_MOTOR_STATUSIN_      CAN_BaseAdr_RX | (_CMDGRP_MOTOR_STATUSIN_ & COMMAND_MASK ) 	// RX
#define CAN_CMDGRP_MOTOR_STATUSOUT_  		CAN_BaseAdr_TX | (_CMDGRP_MOTOR_STATUSOUT_ & COMMAND_MASK ) 		// TX
#define CAN_CMDGRP_CONTROLLER_PARIN_  	CAN_BaseAdr_RX | (_CMDGRP_CONTROLLER_PARIN_ & COMMAND_MASK )  	// RX
#define CAN_CMDGRP_CONTROLLER_PAROUT_  		CAN_BaseAdr_TX | (_CMDGRP_CONTROLLER_PAROUT_  & COMMAND_MASK )	// TX
#define CAN_dummy2				  		0xffff							// NOT used
#define CAN_CMDGRP_MOTOR_ERR_   			CAN_BaseAdr_TX | (_CMDGRP_MOTOR_ERR_ &COMMAND_MASK ) 				// TX 

#define CAN_STDERR						0x666
/*@}*/

/** \defgroup CANmobjs Mapping of CAN identifiers to C167 message objects
 */
/*@{*/
#define MSG_CMDGRP_MOTOR_CONFIN_    	 1 /**< Control message.*/
#define MSG_CMDGRP_MOTOR_CONFOUT_        2 /**< Firmware information.*/
#define MSG_CMDGRP_MOTOR_CTRL_       	 3 /**< Firmware information.*/
#define MSG_DUMMY       				   /**< Firmware information.*/
#define MSG_CMDGRP_MOTOR_STATUSIN_   	 5 /**< Analog input channels 0 - 3.*/
#define MSG_CMDGRP_MOTOR_STATUSOUT_   	 6 /**< Analog input channels 4 - 7.*/
#define MSG_CMDGRP_CONTROLLER_PARIN_     7 /**< Analog input channels 8 - 11.*/
#define MSG_CMDGRP_CONTROLLER_PAROUT_    8 /**< Analog input channels 12 - 14, Temp.*/
#define MSG_ENCODER     				   /**< 2 motor encoders.*/ 
#define MSG__CMDGRP_MOTOR_ERR_    		10 /**< 6 bumpers, 8 remote control buttons.*/

#define MSG_CMDGRP_SYSTEM_BASIC_IN_			4
#define MSG_CMDGRP_SYSTEM_BASIC_OUT_		9
#define MSG_CMDGRP_SYSTEM_MAINTANANCE_IN_	11
#define MSG_CMDGRP_SYSTEM_MAINTANANCE_OUT_	12

#define MSG_STDERR							14
#define MSG_ANSWER							15
/*@}*/

/**
 * Baud rate of CAN bus (kBaud).
 */
#define CAN_BAUD 1000


extern int can_init(int baud_rate, int mc_no);
extern int can_send(char mobj);
extern unsigned char can_receive(void);
extern unsigned char can_count;
extern void can_debug(unsigned char mark, char index);
/**
 * Definitions and declarations for gyro.
 * \todo Integrate gyro definitions properly.
 */
#define rxintno 0x2E                      /**< hardware interrupt # of SSC receive. */
#define txintno 0x2D                      /**< hardware interrupt # of SSC transmit. */

#endif



