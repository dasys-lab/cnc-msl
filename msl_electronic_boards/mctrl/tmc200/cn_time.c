#include "cn_time.h"
#include "cn_serial.h"
#include "cn_commands.h"
#include "cn_controller.h"
#include "main.h"
#include "cn_led.h"
#include "rtc.h"

volatile unsigned long first_try;
volatile unsigned long second_try;

udword cycle_time; // time in ticks of a cycle

udword cycle_start;

void cn_time_init() {

  cycle_time = CYCLE_TIME_DEFAULT;

}

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

// sleep has a resolution of 51,2 microseconds
void cn_sleep(unsigned long microseconds) {
										 // calculate ticks
  unsigned long end_time = cn_getRTC() + ((microseconds * 10) / 512);

  while(cn_getRTC() < end_time);
  
}

void cn_mark_cycle_start() {

  cycle_start = cn_getRTC();

}

void cn_sync_to_cycle() {

    if((cn_getRTC() - cycle_start) > cycle_time) {
	  cn_write_response(GROUP_ERROR_RESPONSE, CMD_CYCLE_OVERTIME, 0, 0);
	  return;
	}

	while((cn_getRTC() - cycle_start) < cycle_time);
	
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
  ledOn();
}

unsigned int cn_get_command_timeout() {
  return T8REL;
}