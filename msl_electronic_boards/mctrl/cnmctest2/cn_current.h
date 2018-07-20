#ifndef CN_CURRENT
#define CN_CURRENT


void init_current(void);
uword getMotorCurrent(ubyte motor);

extern uword cn_motor_current_lookup[];
extern uword motor_current[3];

#endif