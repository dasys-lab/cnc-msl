#ifndef CN_PWM
#define CN_PWM

#include "main.h"

/*************************************************
 * current lookup table only up to 1024 (0x400)! *
 *************************************************/
#define PWM_PERIOD 0x03E8 // hardware value. never use!
#define MAX_PWM    0x03E8 // highest allowed PWM value. (100% power)
#define ZERO_PWM   0x0000 // smallest allowed PWM value. (0% power)

#ifdef VMC

sbit dp_pwm0  = DP7^0;       /**< Direction register 7.0 for PWM0*/
sbit dp_pwm1  = DP7^1;       /**< Direction register 7.1 for PWM1*/
sbit dp_pwm2  = DP7^2;       /**< Direction register 7.1 for PWM2*/
sbit dp_pwm3  = DP7^3;       /**< Direction register 7.1 for PWM3*/

/* PWM Module */
sfr   PWMCON0  = 0xFF30;
sfr   PWMCON1  = 0xFF32;
sfr   PW0      = 0xFE30;
sfr   PW1      = 0xFE32;
sfr   PW2      = 0xFE34;
sfr   PW3      = 0xFE36;
sfr   PWMIC    = 0xF17E;
sfr   PT0      = 0xF030;
sfr   PT1      = 0xF032;
sfr   PT2      = 0xF034;
sfr   PT3      = 0xF036;
sfr   PP0      = 0xF038;
sfr   PP1      = 0xF03A;
sfr   PP2      = 0xF03C;
sfr   PP3      = 0xF03E;
sbit  PTR0     = PWMCON0^0;
sbit  PTR1     = PWMCON0^1;
sbit  PTR2     = PWMCON0^2;
sbit  PTR3     = PWMCON0^3;
sbit  PTI0     = PWMCON0^4;
sbit  PTI1     = PWMCON0^5;
sbit  PTI2     = PWMCON0^6;
sbit  PTI3     = PWMCON0^7;
sbit  PIE0     = PWMCON0^8;
sbit  PIE1     = PWMCON0^9;
sbit  PIE2     = PWMCON0^10;
sbit  PIE3     = PWMCON0^11;
sbit  PIR0     = PWMCON0^12;
sbit  PIR1     = PWMCON0^13;
sbit  PIR2     = PWMCON0^14;
sbit  PIR3     = PWMCON0^15;
sbit  PEN0     = PWMCON1^0;
sbit  PEN1     = PWMCON1^1;
sbit  PEN2     = PWMCON1^2;
sbit  PEN3     = PWMCON1^3;
sbit  PM0      = PWMCON1^4;
sbit  PM1      = PWMCON1^5;
sbit  PM2      = PWMCON1^6;
sbit  PM3      = PWMCON1^7;
sbit  PB01     = PWMCON1^12;
sbit  PS2      = PWMCON1^14;
sbit  PS3      = PWMCON1^15;
sbit  PWMIR    = PWMIC^7;
sbit  PWMIE    = PWMIC^6;

	sbit dir1_pwm0   	= P7^4;		// Pin 7.4: PWM output channel left
	sbit dp_dir1_pwm0  	= DP7^4;    // Direction register 7.4
	sbit dir2_pwm0   	= P7^5;     // Pin 7.5: PWM output channel left
	sbit dp_dir2_pwm0  	= DP7^5;    // Direction register 7.5
	
	sbit dir1_pwm1   	= P7^6;     // Pin 7.6: PWM output channel left
	sbit dp_dir1_pwm1  	= DP7^6;    // Direction register 7.6
	sbit dir2_pwm1   	= P7^7;     // Pin 7.7: PWM output channel left
	sbit dp_dir2_pwm1  	= DP7^7;    // Direction register 7.7
	
	sbit dir1_pwm2   	= P8^1;     // Pin 8.1: PWM output channel left
	sbit dp_dir1_pwm2  	= DP8^1;    // Direction register 8.1
	sbit dir2_pwm2   	= P8^3;     // Pin 8.3: PWM output channel left
	sbit dp_dir2_pwm2  	= DP8^3;    // Direction register 8.3
#endif

//#ifdef TMC
	//#define PWMREG1 CC6_CC_0
	//#define PWMREG2 CC6_CC_1
	//#define PWMREG3 CC6_CC_2
	#define PWMREG1 CC60
	#define PWMREG2 CC61
	#define PWMREG3 CC62
//#endif
/*
#ifdef VMC
	#define PWMREG1 PW0
	#define PWMREG2 PW1
	#define PWMREG3 PW2
#endif
*/
void cn_init_pwm(void);

#endif