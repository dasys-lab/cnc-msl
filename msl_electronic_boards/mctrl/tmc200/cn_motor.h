#ifndef CN_MOTOR
#define CN_MOTOR

#include "main.h"

#include "cn_pwm.h"

#define MOTOR1  0
#define MOTOR2  1
#define MOTOR3  2

#define DIRECTION_CW  0
#define	DIRECTION_CCW 1

#define ENCODER_TICK_BIAS 0x7FFF

void initMotorSystem();
void stopMotor(ubyte motor);
void stopAllMotors();
void setMotorPWM(ubyte motor, ureg pwm);
void setMotorDirection(ubyte motor, bool direction);
bool getMotorDirection(ubyte motor);
void toggleMotorDirection(ubyte motor);
ureg getMotorPWM(ubyte motor);
sword getAndResetMotorTicks(ubyte motor);
void setAllMotorPWM(ureg pwm1, ureg pwm2, ureg pwm3);

extern sdword   encoder_rel[3];
extern sdword   encoder_accu_rel[3];

#endif