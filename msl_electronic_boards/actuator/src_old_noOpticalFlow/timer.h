
#ifndef TIMER_H__
#define TIMER_H__

void     timer_init(void);
void     timer_trigger_callbacks(void);
uint8_t  timer_register(void (*)(void), uint16_t);
uint8_t  timer_deregister(void (*)(void));
uint32_t timer_get_ticks(void);
uint32_t timer_get_ms(void);

#endif
