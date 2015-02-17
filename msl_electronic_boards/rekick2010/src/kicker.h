#ifndef KICKER_H
#define KICKER_H

volatile uint16_t pending_us10;



void kicker_init(void);
void kicker_add_kick_job(uint16_t us10);
void kicker_add_kick_job_forced(uint16_t us10, uint8_t forceVoltage);
void kicker_task_handler(void);

#endif
