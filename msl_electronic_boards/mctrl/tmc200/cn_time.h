#ifndef CN_TIME
#define CN_TIME

#include "main.h"

#define TIME_TICKS_PER_MINUTE 1171875l
#define TIME_TICKS_PER_SECOND 19531l
#define	TIME_TICKS_PER_MILLISECOND 20l
#define cn_musec_to_ticks(musec)   (((long) musec) * 10 / 512)
#define cn_milsec_to_ticks(milsec) (cn_musec_to_ticks((long) milsec * 1000))
                            // 100milsec in ticks
#define CYCLE_TIME_DEFAULT (cn_milsec_to_ticks(100))

extern udword  cycle_time;

void		  cn_time_init();
unsigned long cn_getRTC();
void          cn_sleep(unsigned long msec);
void					cn_mark_cycle_start();
void					cn_sync_to_cycle();
														
void          cn_start_command_timeout();
void          cn_stop_command_timeout();
void          cn_reset_command_timeout();
unsigned int  cn_get_command_timeout();
void          cn_set_command_timeout(unsigned int ticks);

#endif