#ifndef BOOSTER_H
#define BOOSTER_H

#include <inttypes.h>

extern uint32_t last_heartbeat;

void booster_init(void);
void booster_power_enable(void);
void booster_power_disable(void);
void booster_is_power_on(void);
void booster_pwm_enable(void);
void booster_pwm_disable(void);
uint8_t booster_check_pwm_enable(void);
void booster_ctrl(void);
void booster_send_info(void);
void booster_set_max_voltage(uint8_t voltage);
uint8_t booster_can_kick(void);

extern uint8_t max_voltage;
extern uint8_t manual_mode;
extern uint8_t auto_boost;
#endif
