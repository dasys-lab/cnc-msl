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
* Module name.: main                                                           *
********************************************************************************
* Include file: none                                                            *
********************************************************************************
* Project.....: EtherCAN - CAN-Server                                          *
* Filename....: main.h                                                         *
* Authors.....: Sebastian Haas                                                 *
********************************************************************************
* Short descr.: This Modul represent main                                      *
********************************************************************************
* Description.: command arg, decision between server & client mode             *
*                                                                              *
********************************************************************************
*                            History                                           *
********************************************************************************
* Version  Date        Author   Remark                                         *
*                                                                              *
* 01.00    10.04.2003  SH       Initial version                                *
*******************************************************************************/

/***************************************************/
/*               D E F I N E S                     */
#ifndef MAIN_H
#define MAIN_H 1

#define PRG_VERSION "EtherCAN Server V1.1.10"

#define maxSize    255
#define delay      5
#define TCPFRAME   64

#define FDS_STDIN  0

#define TRUE       0
#define FALSE      1

#define CANSRV_GIVEN_PORT		0x0010
#define CANSRV_GIVEN_CONFIG	0x0020
#define CANSRV_GIVEN_IPADDR	0x0040
#define CANSRV_GIVEN_BAUD		0x0080

#define CANSRV_IS_CLIENT		0x00d0
#define CANSRV_IS_SERVER		0x0010

#define CANSRV_CONNECTED		0
#define CANSRV_DISCONNECTED	1
#define CANSRV_CONNECTING		2
#define CANSRV_EXIT					3


#define StartTimer(x, y)		memset(x, 0, sizeof(x)); memset(y, 0, sizeof(y)); gettimeofday(x, NULL); gettimeofday(y, NULL)
#define StopTimer(x, y)			memset(x, 0, sizeof(x)); memset(y, 0, sizeof(y))
#define UpdateTimer(x)			gettimeofday(x, NULL)
#define CheckTimer(x, y, z)	    ((((((y.tv_sec - x.tv_sec) * 1000000) + y.tv_usec) - x.tv_usec)) > (z*1000))

#define OutputMessage(x)		printf("CANSRV(pid %i)[%s:%i:%s]: %s\n", getpid(), __FILE__, __LINE__, __PRETTY_FUNCTION__, x)

/* Macros help using of our buffers */
#define IsBufferFull(x)			(!x.WnR) && (x.iidx == x.oidx)
#define IsBufferEmpty(x)		(x.WnR) && (x.iidx == x.oidx)
#define IsBufferNotEmpty(x)		(!x.WnR) || (x.iidx != x.oidx)
#define IsOob(x)				(x.oob)

// sh watchdog 16.10.2003
void triggerWatchdog();
extern int nWDogFD;

extern unsigned char isBusOff;
struct timeval busoff_ref, busoff_cur;

#endif //MAIN_H
