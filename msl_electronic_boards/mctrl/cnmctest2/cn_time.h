#ifndef CN_TIME
#define CN_TIME

#include "main.h"

//#define TIME_TICKS_PER_MINUTE 1171875l
//#define TIME_TICKS_PER_SECOND 19531l
//#define	TIME_TICKS_PER_MILLISECOND 20l

#define TIME_TICKS_PER_MINUTE      2343750l
#define TIME_TICKS_PER_SECOND      39063l
#define TIME_TICKS_PER_MILLISECOND 39l

                            // 100milsec in ticks
#define CYCLE_TIME_DEFAULT (100 * TIME_TICKS_PER_MILLISECOND)

extern uword  cycle_time;
extern uword  last_cycle_ticks;
extern bool   normal_op;

void		  cn_time_init();
//unsigned long cn_getRTC();
//void          cn_sleep(unsigned long msec);
//unsigned int cn_getTimer();
//void cn_resetTimer();

#ifdef TMC
	#define TIMER T7
#endif
#ifdef VMC
	#define TIMER T6
#endif

#define cn_getTimer()   TIMER
void cn_resetTimer();

//void					cn_mark_cycle_start();
void					cn_sync_to_cycle();
														
void          cn_start_command_timeout();
void          cn_stop_command_timeout();
void          cn_reset_command_timeout();
unsigned int  cn_get_command_timeout();
void          cn_set_command_timeout(unsigned int ticks);

#endif