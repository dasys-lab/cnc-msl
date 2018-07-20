#include "main.h"
#include "cn_pwm.h"

void cn_init_pwm(void) {

#ifdef TMC
	// original TMC200 PWM code
	CTCON   = 0x1300;
	//T12P    = 0x3E7;
	T12P    = PWM_PERIOD;
	T12OF   = 0;
	T13P    = 0x7C;
	CC6MCON = 0x20FF;
	CC6MSEL = 0x222;
	CC60    = 0;
	CC61    = 0;
	CC62    = 0;
	CMP13   = 0;
	CC6MIC  = 0;
	CTCON  |= (1 << 5);
	CTCON  |= (1 << 13);
  
	CTCON  |= 8;         // start T12
	//CTCON  |= 0x0800;  // start T13
	// END original TMC200 PWM code
#endif

#ifdef VMC

	// set PWM period for each PWM generator
	PP0 = PWM_PERIOD;
	PP1 = PWM_PERIOD;
	PP2 = PWM_PERIOD;

	// set initial PWM target
	PW0  = PWM_PERIOD+1; // 0% duty cycle
	PW1  = PWM_PERIOD+1; // 0% duty cycle
	PW2  = PWM_PERIOD+1; // 0% duty cycle

	// disable PWM interrupts
	PWMIE = 0; 
	PIE0  = 0;
	PIE1  = 0;
	PIE2  = 0;
	PIE3  = 0;
	
	PM0  = 0;  // set edge aligned mode
	PM1  = 0;  // set edge aligned mode
	PM2  = 0;  // set edge aligned mode
	PB01 = 0;  // PWM0 and PWM1 outputs are indipendent
	PS2  = 0;  // disable single-shot mode for port 2 (to normal PWM mode)

	// set 1:1 clock input to PWM registers
	PTI0  = 0;
	PTI1  = 0;
	PTI2  = 0;	

	// start PWM generators
	PTR0    = 1;
	PTR1    = 1;
	PTR2    = 1;
	PTR3    = 0; // fourth PWM not used
	
	//PWMCON0 = PWMCON0 | 0x01;  // Start the PT0 counter
	//PWMCON0 = PWMCON0 | 0x02;  // Start the PT1 counter
	//PWMCON0 = PWMCON0 | 0x04;  // Start the PT2 counter

	// set all direction pins open
	dir1_pwm0 = 0;
	dir2_pwm0 = 0;
	dir1_pwm1 = 0;
	dir2_pwm1 = 0;
	dir1_pwm2 = 0;
	dir2_pwm2 = 0;

	// set direction pins as outputs
	dp_dir1_pwm0 = 1;
	dp_dir2_pwm0 = 1;
	dp_dir1_pwm1 = 1;
	dp_dir2_pwm1 = 1;
	dp_dir1_pwm2 = 1;
	dp_dir2_pwm2 = 1;

	// set pwm pins as outputs
	dp_pwm0 = 1;
	dp_pwm1 = 1;
	dp_pwm2 = 1;

	// set PWM output to port
	PEN0  = 1;
	PEN1  = 1;
	PEN2  = 1;
	PEN3  = 0; // fourth PWM not used
	//PWMCON1 = PWMCON1 | 0x01;  // output PWM0 to P7.0
	//PWMCON1 = PWMCON1 | 0x02;  // output PWM1 to P7.1
	//PWMCON1 = PWMCON1 | 0x04;  // output PWM2 to P7.2
	
#endif

}
