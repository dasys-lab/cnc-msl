//////////////////////////////////////////////////////////////////////////////
/// \ingroup utils Different Functions to ease use of AISC167Board
//@{
/// \file ais_utils.h
///
/// \brief 	Different Functions to ease use of AISC167Board
///
/// \author Adam Cwientzek
///
/// \version 0.8
///
/// \date 31.06.2005
///
//////////////////////////////////////////////////////////////////////////////

#ifndef _AIS_UTILS_H_
#define _AIS_UTILS_H_

#include <reg165.h>
#include "ais_typedef.h"


#define _DATA_TO_INT_(a,b)	((a*256)+b)
#define _NumToIndex_(x) x-1


//////////////////////////////////////////////////////////////////////////////
/// I/O Pin green LED
//////////////////////////////////////////////////////////////////////////////
sbit LED_GREEN     = P4^7;

//////////////////////////////////////////////////////////////////////////////
/// I/O Pin red LED
//////////////////////////////////////////////////////////////////////////////
sbit LED_RED       = P4^4;

//////////////////////////////////////////////////////////////////////////////
/// Direction SBit for green LED
//////////////////////////////////////////////////////////////////////////////
sbit DIR_LED_GREEN = DP4^7;

//////////////////////////////////////////////////////////////////////////////
/// Direction SBit for red LED
//////////////////////////////////////////////////////////////////////////////
sbit DIR_LED_RED   = DP4^4;

//////////////////////////////////////////////////////////////////////////////
/// I/O Pin green LED
//////////////////////////////////////////////////////////////////////////////
sbit SW_1     		= P6^6;

//////////////////////////////////////////////////////////////////////////////
/// I/O Pin red LED
//////////////////////////////////////////////////////////////////////////////
sbit SW_2       	= P6^7;

//////////////////////////////////////////////////////////////////////////////
/// Direction SBit for green LED
//////////////////////////////////////////////////////////////////////////////
sbit DIR_SW_1 		= DP6^6;

//////////////////////////////////////////////////////////////////////////////
/// Direction SBit for red LED
//////////////////////////////////////////////////////////////////////////////
sbit DIR_SW_2   	= DP6^7;


//////////////////////////////////////////////////////////////////////////////
/// Define _ON_ as 1
//////////////////////////////////////////////////////////////////////////////
#define _ON_		1

//////////////////////////////////////////////////////////////////////////////
/// Define _OFF_ as 0
//////////////////////////////////////////////////////////////////////////////
#define _OFF_		0


void init_aisc167b();
void led_set_green(VMC_UCHAR_8 status);
void led_set_red(VMC_UCHAR_8 status);
void led_swap_green();
void led_swap_red();
void led_control();

VMC_CHAR_8 get_state_sw1();
VMC_CHAR_8 get_state_sw2();

void sync_cycletime(VMC_UINT_16 desired_cycletime);

#endif /* _AIS_UTILS_H_ */

//@}
