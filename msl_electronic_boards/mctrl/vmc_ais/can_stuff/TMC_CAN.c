/** \file
 * Interface for transmitting data from and to KURT2's microcontroller via CAN bus.
 * $Id: $       
 */

/*
 * The contents of this file are subject to the Mozilla Public
 * License Version 1.1 (the "License"); you may not use this file
 * except in compliance with the License. You may obtain a copy of
 * the License at http://www.mozilla.org/MPL/
 *
 * Software distributed under the License is distributed on an "AS
 * IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
 * implied. See the License for the specific language governing
 * rights and limitations under the License.
 *
 * The Original Code is KURT2 Open Source Firmware and Win32 Software,
 * released March 15, 2001.
 *
 * The Initial Developer of the Original Code is GMD National Research
 * Center for Information Technology.  Portions created by GMD are
 * Copyright (C) 2000 - 2001 GMD National Research Center for
 * Information Technology.  All Rights Reserved.
 *
 * As of July 11, 2001, GMD has been integrated into the Fraunhofer-
 * Gesellschaft. As the new legal entity, Fraunhofer-Gesellschaft has thus
 * taken over all legal relationships involving GMD. Portions created by 
 * Fraunhofer-Gesellschaft are Copyright (C) 2002 Fraunhofer-Gesellschaft.
 * All Rights Reserved.
 * 
 * Contributor(s):
 */

#include <reg167.h>
#include "can_stuff/ais_can.h"
#include "system/ais_utils.h"

#include "aisc167b/ais_led.h"

/**
 * Global buffer for the to be send CAN messages' data bytes.
 */


unsigned char cansend_data[8];

short can_index;
short can_index_last;
short can_done;

struct Can_Receive  can_rec_buf[CANBUFLEN];








//unsigned char can_count=0;
/**
 * Global buffer for the received CAN messages' data bytes.
 */

#define LEN 8
unsigned char canrec_data[8+1];  // one for the length!!


#define C1CSR    (*(canreg *)0x00EF00)        /**< Control/Status Register. */
#define C1IR     (*(canreg *)0x00EF02)        /**< Interrupt Register. */
#define C1BTR    (*(canreg *)0x00EF04)        /**< Bit Timing Register. */
#define C1GMS    (*(canreg *)0x00EF06)        /**< Global Mask Short. */
#define C1UGML   (*(canreg *)0x00EF08)        /**< Global Mask Long. */
#define C1LGML   (*(canreg *)0x00EF0A)        /**< Global Mask Long. */
#define C1UMLM   (*(canreg *)0x00EF0C)        /**< Mask of Last Message. */
#define C1LMLM   (*(canreg *)0x00EF0E)        /**< Mask of Last Message. */

#define C1MOBJ		messageobject *
#define C1MOBJ1  (*(messageobject *)0x00EF10) /**< Message Object 1. */
#define C1MOBJ2  (*(messageobject *)0x00EF20) /**< Message Object 2. */
#define C1MOBJ3  (*(messageobject *)0x00EF30) /**< Message Object 3. */
#define C1MOBJ4  (*(messageobject *)0x00EF40) /**< Message Object 4. */
#define C1MOBJ5  (*(messageobject *)0x00EF50) /**< Message Object 5. */
#define C1MOBJ6  (*(messageobject *)0x00EF60) /**< Message Object 6. */
#define C1MOBJ7  (*(messageobject *)0x00EF70) /**< Message Object 7. */
#define C1MOBJ8  (*(messageobject *)0x00EF80) /**< Message Object 8. */
#define C1MOBJ9  (*(messageobject *)0x00EF90) /**< Message Object 9. */
#define C1MOBJ10 (*(messageobject *)0x00EFA0) /**< Message Object 10. */
#define C1MOBJ11 (*(messageobject *)0x00EFB0) /**< Message Object 11. */
#define C1MOBJ12 (*(messageobject *)0x00EFC0) /**< Message Object 12. */
#define C1MOBJ13 (*(messageobject *)0x00EFD0) /**< Message Object 13. */
#define C1MOBJ14 (*(messageobject *)0x00EFE0) /**< Message Object 14. */
#define C1MOBJ15 (*(messageobject *)0x00EFF0) /**< Message Object 15. */

/**
 * Assign word to CAN register.
 * \param reg CAN register.
 * \param value 16-bit variable.
 */
void setWord(canreg *reg, can_word value) {
  reg->word = value;
}

/**
 * Assign low byte to CAN register.
 * \param reg CAN register.
 * \param value 8-bit variable.
 */
void setLowByte(canreg *reg, can_byte value) {
  (*reg).byte[0] = value;
}

/**
 * Assign (high) byte to CAN register.
 * \param reg CAN register.
 * \param value 8-bit variable.
 */
void setByte(can_byte *reg, can_byte value) {
  *reg = value;
}

/**
 * Read word from CAN register.
 * \param reg CAN register.
 * \param value 16-bit variable.
 */
void getWord(canreg reg, can_word *value) {
  *value = reg.word;
}

/**
 * Initialize on-chip CAN controller.
 * See also chapter 18 of the C167 user's manual.
 * \param baud_rate 50, 125, 250, 500, or 1000 MBaud.
 * \param mc_no 0 or 1, depending whether the interface is used by the first or the second microcontroller.
 * \return success or exception.
 * \retval 0 function was successful.
 * \retval -1 invalid baud_rate.
 */
int can_init(int baud_rate, int mc_no) {
  can_word uar;

  short can_index;



  for (can_index = 0; can_index < CANBUFLEN; can_index++) {
							can_rec_buf[can_index].finished = DONE;
 							can_rec_buf[can_index].occupied = can_index + 1;
 							}
//	can_index = 1;
	can_index_last = 1;
	can_done = 1;



/**
 * Set baudrate.
 */
  setLowByte(&C1CSR, 0x41);       /* INIT:=1, CCE:=1 */
  switch (baud_rate) {
    case 50:
      setWord(&C1BTR, 0x7ac9);    /* BRP:=9, SJW:=3, TSEG1:=a, TSEG2:=7 */
      break;
    case 125:
      setWord(&C1BTR, 0x7ac3);    /* BRP:=3, SJW:=3, TSEG1:=a, TSEG2:=7 */
      break;
    case 250:
      setWord(&C1BTR, 0x7ac1);    /* BRP:=1, SJW:=3, TSEG1:=a, TSEG2:=7 */
      break;
    case 500:
      setWord(&C1BTR, 0x7ac0);    /* BRP:=0, SJW:=3, TSEG1:=a, TSEG2:=7 */
      break;
    case 1000:
      setWord(&C1BTR, 0x25c0);    /* BRP:=0, SJW:=3, TSEG1:=5, TSEG2:=2 */
      break;
    default:
      return -1;
  }
  setLowByte(&C1CSR, 0x01);       /* INIT:=1, CCE:=0 */

/**
 * Initialize CAN related registers.
 */
  setWord(&C1GMS,  0xE0FF);
  setWord(&C1UGML, 0xFFFF);
  setWord(&C1LGML, 0xF8FF);
  setWord(&C1UMLM, 0xFFFF);
  setWord(&C1LMLM, 0xF8FF);

  setWord(&C1MOBJ1.C1MCR,  0xFF7F);
  setWord(&C1MOBJ2.C1MCR,  0xFF7F);
  setWord(&C1MOBJ3.C1MCR,  0xFF7F);
  setWord(&C1MOBJ4.C1MCR,  0xFF7F);
  setWord(&C1MOBJ5.C1MCR,  0xFF7F);
  setWord(&C1MOBJ6.C1MCR,  0xFF7F);
  setWord(&C1MOBJ7.C1MCR,  0xFF7F);
  setWord(&C1MOBJ8.C1MCR,  0xFF7F);
  setWord(&C1MOBJ9.C1MCR,  0xFF7F);
  setWord(&C1MOBJ10.C1MCR, 0xFF7F);
  setWord(&C1MOBJ11.C1MCR, 0xFF7F);
  setWord(&C1MOBJ12.C1MCR, 0xFF7F);
  setWord(&C1MOBJ13.C1MCR, 0xFF7F);
  setWord(&C1MOBJ14.C1MCR, 0xFF7F);
  setWord(&C1MOBJ15.C1MCR, 0xFF7F);

/**
 * Set up message objects for send/receive.
 */
  setWord(&C1MOBJ1.C1MCR, 0x55A9);  /* INTPND:=0, RXIE:=1, TXIE:=1, MSGVAL:=1 */
                                    /* NEWDAT:=0, MSGLST:=0, TXRQ:=0, RMTPND:=0 */
  setByte(&C1MOBJ1.C1MCFG, 0x80);   /* XTD:=0, DIR:=0, DLC:=8 */
  uar = CAN_CMDGRP_MOTOR_CONFIN_ + 16 * mc_no;
  setWord(&C1MOBJ1.C1UAR, (uar << 13) + (uar >> 3));
  setWord(&C1MOBJ1.C1LAR, 0x0000);  /* Lower Message ID: 000000000000000000 */
/////////////////////////////////////////////////////////////////////////////////////////  
  setWord(&C1MOBJ2.C1MCR, 0x59A9);  /* INTPND:=0, RXIE:=1, TXIE:=1, MSGVAL:=1 */
                                    /* NEWDAT:=0, CPUUPD:=1, TXRQ:=0, RMTPND:=0 */
  setByte(&C1MOBJ2.C1MCFG, 0x88);   /* XTD:=0, DIR:=1, DLC:=8 */
  uar = CAN_CMDGRP_MOTOR_CONFOUT_ + 16 * mc_no;
  setWord(&C1MOBJ2.C1UAR, (uar << 13) + (uar >> 3));
  setWord(&C1MOBJ2.C1LAR, 0x0000);  /* Lower Message ID: 000000000000000000 */
/////////////////////////////////////////////////////////////////////////////////////////  
  setWord(&C1MOBJ3.C1MCR, 0x59A9);  /* INTPND:=0, RXIE:=1, TXIE:=1, MSGVAL:=1 */
                                    /* NEWDAT:=0, CPUUPD:=1, TXRQ:=0, RMTPND:=0 */
  setByte(&C1MOBJ3.C1MCFG, 0x80);   /* XTD:=0, DIR:=0, DLC:=8 */
  uar = CAN_CMDGRP_MOTOR_CTRL_ + 16 * mc_no;
  setWord(&C1MOBJ3.C1UAR, (uar << 13) + (uar >> 3));
  setWord(&C1MOBJ3.C1LAR, 0x0000);  /* Lower Message ID: 000000000000000000 */
/////////////////////////////////////////////////////////////////////////////////////////  
  //not used
/////////////////////////////////////////////////////////////////////////////////////////  
  setWord(&C1MOBJ5.C1MCR, 0x59A9);  /* INTPND:=0, RXIE:=1, TXIE:=1, MSGVAL:=1 */
                                    /* NEWDAT:=0, CPUUPD:=1, TXRQ:=0, RMTPND:=0 */
  setByte(&C1MOBJ5.C1MCFG, 0x80);   /* XTD:=0, DIR:=1, DLC:=8 */
  uar = CAN_CMDGRP_MOTOR_STATUSIN_  + 16 * mc_no;
  setWord(&C1MOBJ5.C1UAR, (uar << 13) + (uar >> 3));
  setWord(&C1MOBJ5.C1LAR, 0x0000);  /* Lower Message ID: 000000000000000000 */
/////////////////////////////////////////////////////////////////////////////////////////  
  setWord(&C1MOBJ6.C1MCR, 0x59A9);  /* INTPND:=0, RXIE:=1, TXIE:=1, MSGVAL:=1 */
                                    /* NEWDAT:=0, CPUUPD:=1, TXRQ:=0, RMTPND:=0 */
  setByte(&C1MOBJ6.C1MCFG, 0x88);   /* XTD:=0, DIR:=1, DLC:=8 */
  uar = CAN_CMDGRP_MOTOR_STATUSOUT_ + 16 * mc_no;
  setWord(&C1MOBJ6.C1UAR, (uar << 13) + (uar >> 3));
  setWord(&C1MOBJ6.C1LAR, 0x0000);  /* Lower Message ID: 000000000000000000 */
/////////////////////////////////////////////////////////////////////////////////////////  
  setWord(&C1MOBJ7.C1MCR, 0x59A9);  /* INTPND:=0, RXIE:=1, TXIE:=1, MSGVAL:=1 */
                                    /* NEWDAT:=0, CPUUPD:=1, TXRQ:=0, RMTPND:=0 */
  setByte(&C1MOBJ7.C1MCFG, 0x80);   /* XTD:=0, DIR:=1, DLC:=8 */
  uar = CAN_CMDGRP_CONTROLLER_PARIN_ + 16 * mc_no;
  setWord(&C1MOBJ7.C1UAR, (uar << 13) + (uar >> 3));
  setWord(&C1MOBJ7.C1LAR, 0x0000);  /* Lower Message ID: 000000000000000000 */
 /////////////////////////////////////////////////////////////////////////////////////////  
 setWord(&C1MOBJ8.C1MCR, 0x59A9);  /* INTPND:=0, RXIE:=1, TXIE:=1, MSGVAL:=1 */
                                    /* NEWDAT:=0, CPUUPD:=1, TXRQ:=0, RMTPND:=0 */
  setByte(&C1MOBJ8.C1MCFG, 0x88);   /* XTD:=0, DIR:=1, DLC:=8 */
  uar = CAN_CMDGRP_CONTROLLER_PAROUT_ + 16 * mc_no;
  setWord(&C1MOBJ8.C1UAR, (uar << 13) + (uar >> 3));
  setWord(&C1MOBJ8.C1LAR, 0x0000);  /* Lower Message ID: 000000000000000000 */
/////////////////////////////////////////////////////////////////////////////////////////  
  setWord(&C1MOBJ10.C1MCR, 0x59A9);  /* INTPND:=0, RXIE:=1, TXIE:=1, MSGVAL:=1 */
                                    /* NEWDAT:=0, CPUUPD:=1, TXRQ:=0, RMTPND:=0 */
  setByte(&C1MOBJ10.C1MCFG, 0x88);   /* XTD:=0, DIR:=1, DLC:=8 */
  uar = CAN_CMDGRP_MOTOR_ERR_ + 16 * mc_no;
  setWord(&C1MOBJ10.C1UAR, (uar << 13) + (uar >> 3));
  setWord(&C1MOBJ10.C1LAR, 0x0000);  /* Lower Message ID: 000000000000000000 */


//////////////////////////////////////////////////////////////////////////////////////////  
 setWord(&C1MOBJ4.C1MCR, 0x59A9);  /* INTPND:=0, RXIE:=1, TXIE:=1, MSGVAL:=1 */
                                    /* NEWDAT:=0, CPUUPD:=1, TXRQ:=0, RMTPND:=0 */
  setByte(&C1MOBJ4.C1MCFG, 0x80);   /* XTD:=0, DIR:=1, DLC:=8 */
  uar = CAN_CMDGRP_SYSTEM_BASIC_IN_ + 16 * mc_no;
  setWord(&C1MOBJ4.C1UAR, (uar << 13) + (uar >> 3));
  setWord(&C1MOBJ4.C1LAR, 0x0000);  /* Lower Message ID: 000000000000000000 */
/////////////////////////////////////////////////////////////////////////////////////////  
 setWord(&C1MOBJ9.C1MCR, 0x59A9);  /* INTPND:=0, RXIE:=1, TXIE:=1, MSGVAL:=1 */
                                    /* NEWDAT:=0, CPUUPD:=1, TXRQ:=0, RMTPND:=0 */
  setByte(&C1MOBJ9.C1MCFG, 0x88);   /* XTD:=0, DIR:=1, DLC:=8 */
  uar = CAN_CMDGRP_SYSTEM_BASIC_OUT_ + 16 * mc_no;
  setWord(&C1MOBJ9.C1UAR, (uar << 13) + (uar >> 3));
  setWord(&C1MOBJ9.C1LAR, 0x0000);  /* Lower Message ID: 000000000000000000 */
//////////////////////////////////////////////////////////////////////////////////////////  
 setWord(&C1MOBJ11.C1MCR, 0x59A9);  /* INTPND:=0, RXIE:=1, TXIE:=1, MSGVAL:=1 */
                                    /* NEWDAT:=0, CPUUPD:=1, TXRQ:=0, RMTPND:=0 */
  setByte(&C1MOBJ11.C1MCFG, 0x80);   /* XTD:=0, DIR:=1, DLC:=8 */
  uar = CAN_CMDGRP_SYSTEM_MAINTANANCE_IN_ + 16 * mc_no;
  setWord(&C1MOBJ11.C1UAR, (uar << 13) + (uar >> 3));
  setWord(&C1MOBJ11.C1LAR, 0x0000);  /* Lower Message ID: 000000000000000000 */
/////////////////////////////////////////////////////////////////////////////////////////  
  setWord(&C1MOBJ12.C1MCR, 0x59A9);  /* INTPND:=0, RXIE:=1, TXIE:=1, MSGVAL:=1 */
                                    /* NEWDAT:=0, CPUUPD:=1, TXRQ:=0, RMTPND:=0 */
  setByte(&C1MOBJ12.C1MCFG, 0x88);   /* XTD:=0, DIR:=1, DLC:=8 */
  uar = CAN_CMDGRP_SYSTEM_MAINTANANCE_OUT_ + 16 * mc_no;
  setWord(&C1MOBJ12.C1UAR, (uar << 13) + (uar >> 3));
  setWord(&C1MOBJ12.C1LAR, 0x0000);  /* Lower Message ID: 000000000000000000 */
/////////////////////////////////////////////////////////////////////////////////////////  


/////////////////////////////////////////////////////////////////////////////////////////  
  setWord(&C1MOBJ14.C1MCR, 0x59A9);  /* INTPND:=0, RXIE:=1, TXIE:=1, MSGVAL:=1 */
                                    /* NEWDAT:=0, CPUUPD:=1, TXRQ:=0, RMTPND:=0 */
  setByte(&C1MOBJ14.C1MCFG, 0x88);   /* XTD:=0, DIR:=1, DLC:=8 */
  uar = CAN_STDERR + 16 * mc_no;
  setWord(&C1MOBJ14.C1UAR, (uar << 13) + (uar >> 3));
  setWord(&C1MOBJ14.C1LAR, 0x0000);  /* Lower Message ID: 000000000000000000 */
/////////////////////////////////////////////////////////////////////////////////////////  


  setLowByte(&C1CSR, 0x00);         /* INIT:=0, CCE:=0 */

  return 0;
}

/**
 * Receive CAN message.
 * Move the data bytes of a received CAN message to the global variable canrec_data.
 * \param mobj number of message object buffer to be read.
 * \return success or exception.
 * \retval 0 function was successful.
 * \retval 1 no new CAN message could be read.
 * \retval -1 invalid mobj.
 */
unsigned char can_receive(void) {
  can_word mcr;

//	case MSG_CMDGRP_MOTOR_CONFIN_:
      getWord(C1MOBJ1.C1MCR, &mcr);
      if (mcr & 0x0200) {                 /* NEWDAT set by CAN controller? */

//        rc = 0;
/*
        canrec_data[0] = C1MOBJ1.C1DA0;
        canrec_data[1] = C1MOBJ1.C1DA1;
        canrec_data[2] = C1MOBJ1.C1DA2;
        canrec_data[3] = C1MOBJ1.C1DA3;
        canrec_data[4] = C1MOBJ1.C1DA4;
        canrec_data[5] = C1MOBJ1.C1DA5;
        canrec_data[6] = C1MOBJ1.C1DA6;
        canrec_data[7] = C1MOBJ1.C1DA7;
        canrec_data[LEN] = (C1MOBJ1.C1MCFG >> 4)& 0x0f;
		setWord(&C1MOBJ1.C1MCR, ~0x0200);    // NEWDAT:=0 
*/
		copy2nextBuf( &C1MOBJ1 );

		return 0x50;
      }

//	case MSG_CMDGRP_MOTOR_CTRL_:
      getWord(C1MOBJ3.C1MCR, &mcr);
      if (mcr & 0x0200) {                 /* NEWDAT set by CAN controller? */
	
		copy2nextBuf( &C1MOBJ3);
		return 0x52;
      }

//	case MSG_CMDGRP_MOTOR_STATUSIN_:
      getWord(C1MOBJ5.C1MCR, &mcr);
      if (mcr & 0x0200) {                 /* NEWDAT set by CAN controller? */
	
		copy2nextBuf( &C1MOBJ5);
		return 0x54;
      }

//	case MSG_CMDGRP_CONTROLLER_PARIN_:
      getWord(C1MOBJ7.C1MCR, &mcr);
      if (mcr & 0x0200) {                 /* NEWDAT set by CAN controller? */
	
		copy2nextBuf( &C1MOBJ7);
		return 0x56;
      }
//	case MSG_CMDGRP_SYSTEM_BASIC_IN_:
      getWord(C1MOBJ4.C1MCR, &mcr);
      if (mcr & 0x0200) {                 /* NEWDAT set by CAN controller? */
	
		copy2nextBuf( &C1MOBJ4);
		return 0x10;
      }

//	case MSG_CMDGRP_SYSTEM_MAINTANANCE_IN_:
      getWord(C1MOBJ11.C1MCR, &mcr);
      if (mcr & 0x0200) {                 /* NEWDAT set by CAN controller? */
	
		copy2nextBuf( &C1MOBJ11);
		return 0x1a;
      }

  return 0x00;							// NO message
}

/**
 * Send CAN message.
 * Move the data bytes of the global variable cansend_data to the appropriate message buffer and submit transmission.
 * \param mobj number of message object buffer to be used for send.
 * \return success or exception.
 * \retval 0 function was successful.
 * \retval -1 invalid mobj.
 */
int can_send(char mobj) {

  switch (mobj) {
    case MSG_CMDGRP_MOTOR_CONFOUT_ :
      setWord(&C1MOBJ2.C1MCR, 0xF9FF);    /* NEWDAT:=0, CPUUPD:=1 */
      setByte(&C1MOBJ2.C1DA0, cansend_data[0]);
      setByte(&C1MOBJ2.C1DA1, cansend_data[1]);
      setByte(&C1MOBJ2.C1DA2, cansend_data[2]);
      setByte(&C1MOBJ2.C1DA3, cansend_data[3]);
      setByte(&C1MOBJ2.C1DA4, cansend_data[4]);
      setByte(&C1MOBJ2.C1DA5, cansend_data[5]);
      setByte(&C1MOBJ2.C1DA6, cansend_data[6]);
      setByte(&C1MOBJ2.C1DA7, cansend_data[7]);
      setWord(&C1MOBJ2.C1MCR, 0xE6FF);    /* NEWDAT:=1, CPUUPD:=0, TXRQ:=1 */
      break;
    case MSG_CMDGRP_MOTOR_STATUSOUT_:
      setWord(&C1MOBJ6.C1MCR, 0xF9FF);    /* NEWDAT:=0, CPUUPD:=1 */
      setByte(&C1MOBJ6.C1DA0, cansend_data[0]);
      setByte(&C1MOBJ6.C1DA1, cansend_data[1]);
      setByte(&C1MOBJ6.C1DA2, cansend_data[2]);
      setByte(&C1MOBJ6.C1DA3, cansend_data[3]);
      setByte(&C1MOBJ6.C1DA4, cansend_data[4]);
      setByte(&C1MOBJ6.C1DA5, cansend_data[5]);
      setByte(&C1MOBJ6.C1DA6, cansend_data[6]);
      setByte(&C1MOBJ6.C1DA7, cansend_data[7]);
      setWord(&C1MOBJ6.C1MCR, 0xE6FF);    /* NEWDAT:=1, CPUUPD:=0, TXRQ:=1 */
      break;
    case MSG_CMDGRP_CONTROLLER_PAROUT_:
      setWord(&C1MOBJ8.C1MCR, 0xF9FF);    /* NEWDAT:=0, CPUUPD:=1 */
      setByte(&C1MOBJ8.C1DA0, cansend_data[0]);
      setByte(&C1MOBJ8.C1DA1, cansend_data[1]);
      setByte(&C1MOBJ8.C1DA2, cansend_data[2]);
      setByte(&C1MOBJ8.C1DA3, cansend_data[3]);
      setByte(&C1MOBJ8.C1DA4, cansend_data[4]);
      setByte(&C1MOBJ8.C1DA5, cansend_data[5]);
      setByte(&C1MOBJ8.C1DA6, cansend_data[6]);
      setByte(&C1MOBJ8.C1DA7, cansend_data[7]);
      setWord(&C1MOBJ8.C1MCR, 0xE6FF);    /* NEWDAT:=1, CPUUPD:=0, TXRQ:=1 */
      break;
    case MSG__CMDGRP_MOTOR_ERR_:
      setWord(&C1MOBJ10.C1MCR, 0xF9FF);    /* NEWDAT:=0, CPUUPD:=1 */
      setByte(&C1MOBJ10.C1DA0, cansend_data[0]);
      setByte(&C1MOBJ10.C1DA1, cansend_data[1]);
      setByte(&C1MOBJ10.C1DA2, cansend_data[2]);
      setByte(&C1MOBJ10.C1DA3, cansend_data[3]);
      setByte(&C1MOBJ10.C1DA4, cansend_data[4]);
      setByte(&C1MOBJ10.C1DA5, cansend_data[5]);
      setByte(&C1MOBJ10.C1DA6, cansend_data[6]);
      setByte(&C1MOBJ10.C1DA7, cansend_data[7]);
      setWord(&C1MOBJ10.C1MCR, 0xE6FF);    /* NEWDAT:=1, CPUUPD:=0, TXRQ:=1 */
      break;
    case MSG_CMDGRP_SYSTEM_BASIC_OUT_:
      setWord(&C1MOBJ9.C1MCR, 0xF9FF);    /* NEWDAT:=0, CPUUPD:=1 */
      setByte(&C1MOBJ9.C1DA0, cansend_data[0]);
      setByte(&C1MOBJ9.C1DA1, cansend_data[1]);
      setByte(&C1MOBJ9.C1DA2, cansend_data[2]);
      setByte(&C1MOBJ9.C1DA3, cansend_data[3]);
      setByte(&C1MOBJ9.C1DA4, cansend_data[4]);
      setByte(&C1MOBJ9.C1DA5, cansend_data[5]);
      setByte(&C1MOBJ9.C1DA6, cansend_data[6]);
      setByte(&C1MOBJ9.C1DA7, cansend_data[7]);
      setWord(&C1MOBJ9.C1MCR, 0xE6FF);    /* NEWDAT:=1, CPUUPD:=0, TXRQ:=1 */
      break;
    case MSG_CMDGRP_SYSTEM_MAINTANANCE_OUT_:
      setWord(&C1MOBJ12.C1MCR, 0xF9FF);    /* NEWDAT:=0, CPUUPD:=1 */
      setByte(&C1MOBJ12.C1DA0, cansend_data[0]);
      setByte(&C1MOBJ12.C1DA1, cansend_data[1]);
      setByte(&C1MOBJ12.C1DA2, cansend_data[2]);
      setByte(&C1MOBJ12.C1DA3, cansend_data[3]);
      setByte(&C1MOBJ12.C1DA4, cansend_data[4]);
      setByte(&C1MOBJ12.C1DA5, cansend_data[5]);
      setByte(&C1MOBJ12.C1DA6, cansend_data[6]);
      setByte(&C1MOBJ12.C1DA7, cansend_data[7]);
      setWord(&C1MOBJ12.C1MCR, 0xE6FF);    /* NEWDAT:=1, CPUUPD:=0, TXRQ:=1 */
      break;
    case MSG_STDERR:
      setWord(&C1MOBJ14.C1MCR, 0xF9FF);    /* NEWDAT:=0, CPUUPD:=1 */
      setByte(&C1MOBJ14.C1DA0, cansend_data[0]);
      setByte(&C1MOBJ14.C1DA1, cansend_data[1]);
      setByte(&C1MOBJ14.C1DA2, cansend_data[2]);
      setByte(&C1MOBJ14.C1DA3, cansend_data[3]);
      setByte(&C1MOBJ14.C1DA4, cansend_data[4]);
      setByte(&C1MOBJ14.C1DA5, cansend_data[5]);
      setByte(&C1MOBJ14.C1DA6, cansend_data[6]);
      setByte(&C1MOBJ14.C1DA7, cansend_data[7]);
      setWord(&C1MOBJ14.C1MCR, 0xE6FF);    /* NEWDAT:=1, CPUUPD:=0, TXRQ:=1 */
      break;

 // USE: setting the databytes[]  + can_send(MSG_ANSWER);
    case MSG_ANSWER:
      setWord(&C1MOBJ15.C1MCR, 0xF9FF);    /* NEWDAT:=0, CPUUPD:=1 */
      setByte(&C1MOBJ15.C1DA0, cansend_data[0]);
      setByte(&C1MOBJ15.C1DA1, cansend_data[1]);
      setByte(&C1MOBJ15.C1DA2, cansend_data[2]);
      setByte(&C1MOBJ15.C1DA3, cansend_data[3]);
      setByte(&C1MOBJ15.C1DA4, cansend_data[4]);
      setByte(&C1MOBJ15.C1DA5, cansend_data[5]);
      setByte(&C1MOBJ15.C1DA6, cansend_data[6]);
      setByte(&C1MOBJ15.C1DA7, cansend_data[7]);
      setWord(&C1MOBJ15.C1MCR, 0xE6FF);    /* NEWDAT:=1, CPUUPD:=0, TXRQ:=1 */
      break;

    default:
      return -1;
  }

  return 0;
}



short copy2nextBuf(messageobject*  Received) {
short index;

	if ((index = nextCANindex())) {	can_rec_buf[index-1].can_buf = *Received;}

	setWord(&Received->C1MCR, ~0x0200);    // NEWDAT:=

//if (can_send( MSG__CMDGRP_MOTOR_ERR_ )) while(1){led_swap_red();};


	return index;
}


short nextCANindex(void) {
short can_index;

	can_index = (can_index_last)%CANBUFLEN;  // can_index_last != 0

	if (!can_rec_buf[can_index].occupied ) return OCCUPIED;
	
	can_rec_buf[can_index].occupied = OCCUPIED;

//if (can_send( MSG_CMDGRP_SYSTEM_BASIC_OUT_ )) while(1){led_swap_green();};

return (can_index_last = can_index + 1 );
}



typedef struct CmdStruct {
// dummy
int dummy;
};

short can_answer(void) {
short can_index;
//struct CmdStruct Command;

//if (can_send( MSG_ANSWER )) while(1){led_swap_red();};

	can_index = (can_done)%CANBUFLEN;  // can_done != 0

	if ( can_rec_buf[can_index].occupied ) {

// delete receive buffer
		
		return DONE; 
		}

////////////////////////////////////////////////////////////////////////////////////////////
// proceed with the common command structure
/*
typedef struct CmdStruct {
	VMC_UCHAR_8 rxaddr, txaddr;
    VMC_UCHAR_8 count, retcount, sysinf;
	VMC_UCHAR_8 datalen, channel;
	VMC_UCHAR_8 cmdgrp, cmd;
	VMC_UCHAR_8 data[_CMDB_MAXDATA_];
};
*/











////////////////////////////////////////////////////////////////////////////////////////////

	// send an answer for this messageobject

/*
  canreg    C1MCR;      /* < Message Control Register. 
  canreg    C1UAR;      /**< Upper Arbitration Register.
  canreg    C1LAR;      /**< Lower Arbitration Register. 
  can_byte  C1MCFG;     /**< Message Configuration Register. */

      setWord(&C1MOBJ15.C1MCR, 0xF9FF);    /* NEWDAT:=0, CPUUPD:=1 */
  setByte(&C1MOBJ15.C1DA0, can_rec_buf[can_index].can_buf.C1DA0);      /**< Data Area. */
  setByte(&C1MOBJ15.C1DA1, can_rec_buf[can_index].can_buf.C1DA1);      /**< Data Area. */
  setByte(&C1MOBJ15.C1DA2, can_rec_buf[can_index].can_buf.C1DA2);      /**< Data Area. */
  setByte(&C1MOBJ15.C1DA3, can_rec_buf[can_index].can_buf.C1DA3);      /**< Data Area. */
  setByte(&C1MOBJ15.C1DA4, can_rec_buf[can_index].can_buf.C1DA4);      /**< Data Area. */
  setByte(&C1MOBJ15.C1DA5, can_rec_buf[can_index].can_buf.C1DA5);      /**< Data Area. */
  setByte(&C1MOBJ15.C1DA6, can_rec_buf[can_index].can_buf.C1DA6);      /**< Data Area. */
  setByte(&C1MOBJ15.C1DA7, can_rec_buf[can_index].can_buf.C1DA7);      /**< Data Area. */
     setWord(&C1MOBJ15.C1MCR, 0xE6FF);    /* NEWDAT:=1, CPUUPD:=0, TXRQ:=1 */

 // can_rec_buf[can_index].C1RESVD;    /**< Reserved. */


//			if (can_send( MSG_CMDGRP_SYSTEM_MAINTANANCE_OUT_ )) while(1){led_swap_red();};


	can_rec_buf[can_index].occupied = FREE;
	can_done = can_index + 1;
	
    return BUSY;
}
