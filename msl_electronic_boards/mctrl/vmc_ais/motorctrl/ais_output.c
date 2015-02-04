/////////////////////////////////////////////////////////////////////////////
/// \defgroup Message output
//@{
/// \file ais_output.c
///
/// \brief Module for Status/Error Output
///
///
///
///
/// \author Adam Cwientzek
///
/// \version 0.1
///
/// \date 04.10.2006
///
//////////////////////////////////////////////////////////////////////////////


#include "ais_typedef.h"
#include "motorctrl/ais_output.h"
#include "aisc167b/ais_led.h"

void output_msg(VMC_UCHAR_8 message) {
 switch ( message ) {
   case _OMSG_RUN_:		ledseq_set_break(_LED_BREAK_SHORT_);
   						ledseq_set_red(_LED_SP_OFF_,0);
						ledseq_set_green(_LED_SP_SLOW_,1);
						break;

   case _OMSG_EMSTOP_:	ledseq_set_break(_LED_BREAK_LONG_);
   						ledseq_set_red(_LED_SP_SLOW_,2);
						ledseq_set_green(_LED_SP_FAST_,1);
						break;

   case _OMSG_ERRBUF_:	ledseq_set_break(_LED_BREAK_SHORT_);
   						ledseq_set_red(_LED_SP_FAST_,5);
						ledseq_set_green(_LED_SP_FAST_,0);
						break;


   // default, unknown state
   default:				ledseq_set_break(_LED_BREAK_SHORT_);
   						ledseq_set_red(_LED_SP_FAST_,1);
						ledseq_set_green(_LED_SP_FAST_,1);
 }
}


//@}
