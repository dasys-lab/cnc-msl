#ifndef CN_MOTOR
#define CN_MOTOR

#include "main.h"

#include "cn_pwm.h"

#define MOTOR1  0
#define MOTOR2  1
#define MOTOR3  2

#ifdef TMC
	#define MOTOR1_ENCODER GPT1_TIMER_2
	#define MOTOR2_ENCODER GPT1_TIMER_3
	#define MOTOR3_ENCODER GPT1_TIMER_4
#endif
#ifdef VMC
	#define MOTOR1_ENCODER T2
	#define MOTOR2_ENCODER T3
	#define MOTOR3_ENCODER T4
#endif


#define DIRECTION_CW  -1
#define	DIRECTION_CCW 1
#define DIRECTION_FREE 0


#define ENCODER_TICK_BIAS 0x7FFF

#ifdef VMC
	// encoder input directions
	sbit dp_T3EUD = DP3^4;      /**< Direction register 3.4 for T3EUD*/ 
	sbit dp_T4IN  = DP3^5;       /**< Direction register 3.5 for T4IN*/ 
	sbit dp_T3IN  = DP3^6;       /**< Direction register 3.6 for T3IN*/ 
	sbit dp_T2IN  = DP3^7;       /**< Direction register 3.7 for T2IN*/

	// Direction register for Port 2.0 - 2.5
	sbit dirP20      = DP2^0;
	sbit dirP21      = DP2^1;
	sbit dirP22      = DP2^2;
	sbit dirP23      = DP2^3;
	sbit dirP24      = DP2^4;
	sbit dirP25      = DP2^5;

	// H-Bridge Error Signals on Port 2.0 - 2.5
	sbit ERRA1    = P2^0;
	sbit ERRA2    = P2^1;
	sbit ERRB1    = P2^2;
	sbit ERRB2    = P2^3;
	sbit ERRC1    = P2^4;
	sbit ERRC2    = P2^5;

#endif

void initMotorSystem();
void stopMotor(ubyte motor);
void stopAllMotors();
void setMotorPWM(ubyte motor, ureg pwm);
void setMotorDirection(ubyte motor, sbyte direction);
//void setMotorDirectionExt(ubyte motor, sbyte direction);
sbyte getMotorDirection(ubyte motor);
void toggleMotorDirection(ubyte motor);
ureg getMotorPWM(ubyte motor);
sword getAndResetMotorTicks(ubyte motor);
void setAllMotorPWM(ureg pwm1, ureg pwm2, ureg pwm3);

#ifdef VMC
bool  isOvertemp(ubyte motor);
#endif

extern sdword   encoder_rel[3];
extern sdword   encoder_accu_rel[3];

#endif