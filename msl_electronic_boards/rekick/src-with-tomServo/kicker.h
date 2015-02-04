#ifndef KICKER_H
#define KICKER_H

void kicker_init(void);
void kicker_add_kick_job(uint8_t ms);
void kicker_add_kick_job_forced(uint8_t ms, uint8_t forceVoltage);
void kicker_task_handler(void);
void kicker_rotate_handler(void);
void kicker_add_rotate_job(uint8_t num);
void kicker_set_servo_pos(uint8_t num, uint16_t val);
uint8_t kicker_get_pos(void);

// only for tests (do not use this for the normal system)
void kicker_toggle_interlock(void);
void kicker_rotate_servo(uint8_t num);

#endif
