/*******************************************************************************
*                                                                              *
*      Copyright (c) 2003 by EMS Dr. Thomas Wuensche                           *
*                                                                              *
*                  - All rights reserved -                                     *
*                                                                              *
* This code is provided "as is" without warranty of any kind, either           *
* expressed or implied, including but not limited to the liability             *
* concerning the freedom from material defects, the fitness for parti-         *
* cular purposes or the freedom of proprietary rights of third parties.        *
*                                                                              *
********************************************************************************
* Module name.: can                                                            *
********************************************************************************
* Include file: can.h                                                          *
********************************************************************************
* Project.....: EtherCAN - CAN-Server                                          *
* Filename....: can.c                                                          *
* Authors.....: Sebastian Haas                                                 *
********************************************************************************
* Short descr.: This Modul represent the interface to the CAN-Bus              *
********************************************************************************
* Description.: Take CAN-Messages and convert it to TCP-Frames and sending     *
*                                                                              *
********************************************************************************
*                            History                                           *
********************************************************************************
* Version  Date        Author   Remark                                         *
*                                                                              *
* 01.00    10.04.2003  SH       Initial version                                *
*******************************************************************************/

#ifndef CAN_H
#define CAN_H

/******************************************************************************/
/*     I N C L U D E S
*/
#include <linux/cpc.h>

/******************************************************************************/
/*     D E F I N E S
*/

/* size of buffer holding converted cpc message (cpc->tcp)*/
#define UDP_BUF_MAX 1500

typedef struct _UDP_FRAMES{
	unsigned char length; // length
	unsigned char data[65]; // data
} UDP_FRAMES;

typedef struct _UDP_FRAME{
	unsigned int iidx; // input pointer
	unsigned int oidx; // output pointer
	unsigned char WnR; // Write not Read
	UDP_FRAMES *udpFrames; // pointer to buffer
} UDP_FRAME;

/******************************************************************************/
/*     F U N C T I O N   A N D  V A R I A B L E  D E C L A R A T I O N S
*/

/* Make some init things*/
int initCan(const char *dev);
void printCpcMessage(CPC_MSG_T *srcBuf, unsigned char from);

/* convert a cpc message to an tcp frame and store it in a buffer*/
int convertAndStore_CpcMessage(const CPC_MSG_T *srcBuf, int len);

/******************/
/* Select handler */
/******************/

/* if we can write, send messages from buffer*/
void processWriteUdp();
/* reading cpc messages */
void processReadCpc(int can_handle);

extern CPC_MSG_T lastWrittenParams;

#endif
