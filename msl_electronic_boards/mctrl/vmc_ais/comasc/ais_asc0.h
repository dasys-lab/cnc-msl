//////////////////////////////////////////////////////////////////////////////
/// \ingroup comasc Hardware RS232 interface
//@{
/// \file ais_asc0.h
///
/// \brief 	Header File for ASC0 Functions
///
/// \author Adam Cwientzek
///
/// \version 0.8
///
/// \date 31.06.2005
///
//////////////////////////////////////////////////////////////////////////////

#ifndef _AIS_ASC0_H_
#define _AIS_ASC0_H_

#include "ais_typedef.h"


//////////////////////////////////////////////////////////////////////////////
/// special Characters for Packet Start, Stop and Quote-Character
///  use _ASC_P_START_ for Identification of Packet Start
///      _ASC_P_END_   for Identification of Packet End
///		 _ASC_QUOTE_   Quote for sending characters that have same Val as
///					   the shown special Characters
//////////////////////////////////////////////////////////////////////////////
#define _ASCCH_START_	0x7B
#define _ASCCH_END_		0x7D
#define _ASCCH_QUOTE_	0x5C

// packet count, command group byte, command byte
#define ASC_HEAD_SIZE	3


//////////////////////////////////////////////////////////////////////////////
/// Constants for Communication States
//////////////////////////////////////////////////////////////////////////////
#define _ASCST_WAIT_	0
#define _ASCST_START_	1
#define _ASCST_RCVD_ 	2
#define _ASCST_QT_ 		3



//////////////////////////////////////////////////////////////////////////////
/// Maximal Data Length
//////////////////////////////////////////////////////////////////////////////
#define _ASC_MAX_LEN_ 	   30	// 30


char putchar (char x);

void asc0_init(void);
void asc0_rcv_interrupt(void);
void asc0_trx_interrupt(void);
void asc0_trxbuff_interrupt(void);
void asc0_start_trx(char all);

#endif /* _AIS_ASC0_H_ */

//@}
