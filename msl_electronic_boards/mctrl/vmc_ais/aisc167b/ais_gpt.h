//////////////////////////////////////////////////////////////////////////////
/// \ingroup gpt General Purpose Time Units
//@{
/// \file ais_gpt.h
///
/// \brief 	Header File for the GPT functions
///
/// \author Jan Paulus
///
/// \version 0.8
///
/// \date 24.08.2005
///
//////////////////////////////////////////////////////////////////////////////

#ifndef _AIS_GPT_H_
#define _AIS_GPT_H_

#include <reg167.h>

#include "ais_typedef.h"

#define half 0x7FFF					// half of the full 16 Bit Timer

sbit dp_T3EUD  = DP3^4;      /**< Direction register 3.4 for T3EUD*/ 
sbit dp_T4IN  = DP3^5;       /**< Direction register 3.5 for T4IN*/ 
sbit dp_T3IN  = DP3^6;       /**< Direction register 3.6 for T3IN*/ 
sbit dp_T2IN  = DP3^7;       /**< Direction register 3.7 for T2IN*/ 

void init_cycletime_counter(void);

long get_cycle_time(void);
long get_cycle_time_part(void);
void init_encoder(VMC_UCHAR_8 motor_id);
long get_encoderticks(VMC_UCHAR_8 id);


#endif /* _AIS_GPT_H_ */

//@}
