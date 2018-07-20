#ifndef CN_PWM
#define CN_PWM

#define ZERO_PWM  0x0000
/*************************************************
 * current lookup table only up to 1024 (0x400)! *
 *************************************************/
#define MAX_PWM   0x03E8

void cn_init_pwm(void);

#endif