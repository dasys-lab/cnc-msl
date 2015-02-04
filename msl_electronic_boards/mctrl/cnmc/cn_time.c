#include "cn_time.h"
#include "cn_serial.h"
#include "cn_commands.h"
#include "cn_controller.h"
#include "main.h"
#include "cn_led.h"
#include "rtc.h"

volatile unsigned long first_try;
volatile unsigned long second_try;

uword cycle_time; // time in ticks of a cycle
uword last_cycle_ticks;

void cn_time_init() {

	cycle_time = CYCLE_TIME_DEFAULT;
	
	/*
	T14REL = 0xFFFF;
	T14	=	0xFFFF;
	*/
	
#ifdef TMC

	T78CON_T7R  = 0;      // stop T7
	T78CON     |= 0x0006; // prescaler, F=39.06kHz, 25.6us, T=1.68s
	T78CON_T7M  = 0;      // timer mode
	T7REL       = 0x0000;
	T7          = 0x0000;
	T78CON_T7R  = 1;      // start T7

#endif
#ifdef VMC

	T6R = 0;
	//T6CON_T6M  = 0; // timer mode
	//T6CON_T6UD = 0; // count up
	//T6CON_T6I  = 6; // F=39.06kHz, Resolution 25.6us, T=1.68s
	T6CON = 0x0007;   // T6, F=39.06kHz, Resolution 25.6us, T=1.68s, Timer Mode, Count Up
	T6    = 0x0000;   // Start T6 from Zero
	T6R   = 1;        // Start Timer

#endif

}

/*
__inline unsigned int cn_getTimer() {

	return TIMER;

}

__inline void cn_resetTimer() {

	TIMER = 0x0000;

}
*/

void cn_resetTimer() {

	last_cycle_ticks = TIMER;
	TIMER = 0x0000;

}

/*
unsigned long cn_getRTC() {

	first_try  = RTC_GetTime();  
	second_try = RTC_GetTime();

	if(first_try > second_try) {
		// there was an internal overflow
		// RTCL -> RTCH (see RTC_GetTime())
		second_try = RTC_GetTime();
	}

	return second_try;

}
*/

// sleep has a resolution of 51,2 microseconds
/*
void cn_sleep(unsigned long microseconds) {
										 // calculate ticks
	unsigned long end_time = cn_getRTC() + ((microseconds * 10) / 512);

	while(cn_getRTC() < end_time);
  
}
*/

/*
void cn_mark_cycle_start() {

	//cycle_start = cn_getRTC();
	cycle_start = cn_getTimer();

}
*/

void cn_sync_to_cycle() {

/*
    if((cn_getRTC() - cycle_start) > cycle_time) {
	  cn_write_response(GROUP_ERROR_RESPONSE, CMD_CYCLE_OVERTIME, 0, 0);
	  return;
	}

	while((cn_getRTC() - cycle_start) < cycle_time);
*/


	if(cn_getTimer() > cycle_time) {
		cn_write_response(GROUP_ERROR_RESPONSE, CMD_CYCLE_OVERTIME, 0, 0);
		return;
	}

	
	while(cn_getTimer() < cycle_time);
	
}

void cn_start_command_timeout() {
	CC2_StartTimer(CC2_TIMER_8);
}

void cn_stop_command_timeout() {
	CC2_StopTimer(CC2_TIMER_8);
}

void cn_set_command_timeout(unsigned int ticks) {
	CC2_SetReload(CC2_TIMER_8, ticks);
}

void cn_reset_command_timeout() {
	CC2_TIMER_8 = T8REL;
	cn_controller_reset_error(CN_CONTROLLER_ERROR_TIMEOUT);
	greenLedOn();
#ifdef VMC
	//redLedOff();
#endif
}

unsigned int cn_get_command_timeout() {
  return T8REL;
}