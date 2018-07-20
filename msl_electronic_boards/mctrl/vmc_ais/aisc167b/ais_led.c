/////////////////////////////////////////////////////////////////////////////
/// \defgroup led LED Signals
//@{
/// \file ais_led.c
///
/// \brief Module for Status/Error Infos via LED
///
///
///
///
/// \author 
///
/// \version 0.1
///
/// \date 09.08.2006
///
//////////////////////////////////////////////////////////////////////////////

#include "aisc167b/ais_led.h"
#include "system/ais_utils.h"


VMC_UINT_16 speed_rd;
VMC_UINT_16 speed_gr;
VMC_CHAR_8 count_rd, counter_rd;
VMC_CHAR_8 count_gr, counter_gr;
VMC_UINT_16 breaktime;
VMC_CHAR_8 act_seq, cyclus;
VMC_UINT_16 timer;

//////////////////////////////////////////////////////////////////////////////
/// \brief init LED output module
//////////////////////////////////////////////////////////////////////////////
void ledseq_init() {
	breaktime	= _LED_BREAK_LONG_;

	speed_rd 	= _LED_SP_ON_;
	speed_gr 	= _LED_SP_SLOW_;

	count_rd 	= 1;
	count_gr 	= 1;

	counter_rd 	= 1;
	counter_gr 	= 1;

	timer 		= 0;

	act_seq		= 1;
	cyclus 		= 0;

	led_set_red(0);
	led_set_green(0);
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief set break time after blink sequence
//////////////////////////////////////////////////////////////////////////////
void ledseq_set_break(VMC_CHAR_8 led_break) {
	breaktime = led_break * _LED_SP_BASE_;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief set sequence data for red led
//////////////////////////////////////////////////////////////////////////////
void ledseq_set_red(VMC_CHAR_8 speed, VMC_CHAR_8 count) {
	speed_rd 	= speed * _LED_SP_BASE_;
	count_rd	= count;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief set sequence data for green led
//////////////////////////////////////////////////////////////////////////////
void ledseq_set_green(VMC_CHAR_8 speed, VMC_CHAR_8 count) {
	speed_gr 	= speed * _LED_SP_BASE_;
	count_gr	= count;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief control / show blink sequence
//////////////////////////////////////////////////////////////////////////////
void ledseq_show() {

	switch ( act_seq ) {
	  // green sequence
	  case 1: if ( counter_gr == 0 ) { // sequenze ended
	  		    counter_rd = count_rd; // Set red counter to blink-count
				  counter_gr 	= count_gr;
				  cyclus 	  	= 1;
				  timer   		= 0;
				  act_seq		= 2;
		  	  } else { // still in sequenze
		  	    timer++;
		  	    if ( timer >= speed_gr ) { // change something
		  	     if ( cyclus ) { // led was on
		  	      cyclus 	 	= 0; // switch off led
		  	      counter_gr--;      // on blink sequenze finished
		  	      timer 		= 0;
				 } else { // led was off
				  cyclus 		= 1; // switch on led
				  timer 		= 0;
			 	 }
				}
				led_set_green(cyclus);
		      }
		      break;
	  // red sequence
	  case 2: if ( counter_rd == 0 ) { // sequenze ended
				  timer   		= 0;
				  act_seq		= 3;
		  	  } else { // still in sequenze
		  	    timer++;
		  	    if ( timer >= speed_rd ) { // change something
		  	     if ( cyclus ) { // led was on
		  	      cyclus 	 	= 0; // switch off led
		  	      counter_rd--;      // on blink sequenze finished
		  	      timer 		= 0;
				 } else { // led was off
				  cyclus 		= 1; // switch on led
				  timer 		= 0;
			 	 }
				}
				led_set_red(cyclus);
		      }
		      break;
	  // break sequence
	  case 3: timer++;
	  		  if ( timer >= breaktime ) {
				  counter_gr 	= count_gr;
				  cyclus 	  	= 0;
				  timer   		= 0;
				  act_seq		= 1;
			  }

	} // switch
}
//////////////////////////////////////////////////////////////////////////////


//@}


