#ifndef BEAGLEPWM_H_
#define BEAGLEPWM_H_

#include "BeagleGPIO.h"
#include <stdint.h>


#define NUM_PWMS 3
#define PWM_REGLEN 32


class BeaglePWM
{

	public:
		BeaglePWM(void *pwm_addr);
		virtual ~BeaglePWM();

		void setDutyCycle(int pin, unsigned long us);
		void setPeriod(int pin, unsigned long us);
		void setRunState(int pin, unsigned long us);

	private:
		int memFd; /* file descriptor for /dev/mem */
		uint32_t *pwmRegs[NUM_PWMS]; /** 3 mmaped pwm registers */

		// use singleton?
		//static BeaglePWM *getInstance();
};

#endif /* BEAGLEPWM_H_ */
