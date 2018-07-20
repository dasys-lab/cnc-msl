#include "cn_logging.h"
#include "cn_time.h"
#include "main.h"

#ifndef CN_NO_LOG
typedef struct {
	ubyte  event;
	uword  subject;
	udword timestamp;
	udword data[3];
} cn_log_event;

// this is the linker/system maximum!
#define CN_LOG_MAX_EVENTS 485

cn_log_event cn_log_buffer[CN_LOG_MAX_EVENTS];
uword        cn_log_buffer_read;
uword        cn_log_buffer_write;
#endif

void init_logging() {

#ifndef CN_NO_LOG

	cn_log_buffer_read  = 0;
	cn_log_buffer_write = 0;
	
#endif

}

#ifndef CN_NO_LOG
// never use this directly, use cn_log() macro!
void _cn_log(ubyte event, uword subject, udword data[3]) {

    uword i;
	cn_log_event my_event;
	
	my_event = cn_log_buffer[cn_log_buffer_write++];
	
	if(cn_log_buffer_write >= CN_LOG_MAX_EVENTS) {
		cn_log_buffer_write = 0;
	}
	
	my_event.timestamp = cn_getRTC();
	
	my_event.event   = event;
	my_event.subject = subject;
	for(i = 0; i < 3; ++i) {
		my_event.data[i] = data[i];
	}

}
#endif