//////////////////////////////////////////////////////////////////////////////
/// \ingroup comasc Serial communication Module
//@{
/// \file ais_asccom.h
///
/// \brief 	Header File for serial communication functions
///
/// \author Adam Cwientzek
///
/// \version 0.8
///
/// \date 30.10.2006
///
//////////////////////////////////////////////////////////////////////////////

#include "system/ais_cmdbuff.h"

//////////////////////////////////////////////////////////////////////////////
/// Constants for Packet positions of Commands etc.
//////////////////////////////////////////////////////////////////////////////
#define _ASC0_PKT_MINSIZE_ 4    // Min Pkt is COUNT, GRP, CMD and CRC
#define _ASCPK_COUNT_	   0 	// Byte for Packet Count
#define _ASCPK_GRP_		   1 	// Byte for Command Group
#define _ASCPK_CMD_		   2 	// Byte for Command
#define _ASCPK_DATA_	   3 	// Byte for Datastart
// CRC is last, after Data



#define ASC_BUF_OUT		0
#define ASC_BUF_IN		1
#define ASC_SEND_ONE	0
#define ASC_SEND_ALL	1

void init_asccom(void);
void asccom_senddata(char all);
char asccom_buffer_overrun(char buffer);
char asccom_buffer_r_overrun(char buffer);
char asccom_newcmd(void);
void asccom_empty_buffer(char buffer);
char asccom_getcmd(struct CmdStruct *Cmd);
char asccom_storecmd(struct CmdStruct *Cmd);
void asccom_quotewrite(char buffer, char data);
void asccom_disable_rcv();
void asccom_enable_rcv();


void asccom_debug(VMC_UCHAR_8 *string);
char RS232_asccom_storecmd(struct CmdStruct *Cmd);



//@}


