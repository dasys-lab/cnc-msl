
#ifndef LOGGER_H
#define LOGGER_H

#define LOG_OFF		0
#define LOG_ERROR	1
#define LOG_WARNING	2
#define LOG_INFO	3
#define	LOG_VERBOSE	4

extern uint8_t loglevel;

void log_if(uint8_t thislevel, char *str);

#endif /* LOGGER_H */
