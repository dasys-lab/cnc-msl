/////////////////////////////////////////////////////////////////////////////
/// \ingroup led LED Signals
//@{
/// \file ais_led.h
///
/// \brief Module for Status/Error Infos via LED
///
///
///
///
/// \author Pascal Langenberg
///
/// \version 0.1
///
/// \date 09.08.2006
///
//////////////////////////////////////////////////////////////////////////////

#ifndef _AIS_LED_H_
#define _AIS_LED_H_

#include "ais_typedef.h"


#define _LED_BREAK_NO_			0	
#define _LED_BREAK_SHORT_		2
#define _LED_BREAK_LONG_		5

#define _LED_SP_SLOW_			8
#define _LED_SP_FAST_		 	2	
#define _LED_SP_ON_			  	100 
#define _LED_SP_OFF_		  	0

#define _LED_SP_BASE_			2



// call this function every programm loop to activate led output signaling
void ledseq_init();
void ledseq_show();
void ledseq_set_break(VMC_CHAR_8 led_break);
void ledseq_set_red(VMC_CHAR_8 speed, VMC_CHAR_8 count);
void ledseq_set_green(VMC_CHAR_8 speed, VMC_CHAR_8 count);



#endif //LED_H



//@}
