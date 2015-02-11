#ifndef CN_LOG
#define CN_LOG

#include "main.h"

#define LOG_EVENT_RECV_PACKET      0 // subject: group in high and command in low byte
#define LOG_EVENT_SEND_PACKET      1 // subject: group in high and command in low byte
#define LOG_EVENT_CTRL_ERROR_SET   2 // subject: the error type(s)  data: none
#define LOG_EVENT_CTRL_ERROR_RESET 3 // subject: the error type(s)  data: none
#define LOG_EVENT_SET_FINAL_PWM    4 // subject: effective controller mode  data: signed pwm values
#define LOG_EVENT_CTRL_CMD_TIMEOUT 5 // subject: effective controller mode  data: none
#define LOG_EVENT_SENSOR_UPDATE    6 // subject: sensor (current, voltage, ticks etc.) data: sensor data
#define LOG_EVENT_BEGIN_OF_CYCLE   7 // subject: none  data: none
#define LOG_EVENT_END_OF_WORK      8 // subject: none  data: none

#ifdef CN_NO_LOG
	#define cn_log(e, s, d)
#else
	#define cn_log(e, s, d)	_cn_log((e), (s), (d))
#endif

void init_logging(void);
void _cn_log(ubyte event, uword subject, udword data[3]);

#endif