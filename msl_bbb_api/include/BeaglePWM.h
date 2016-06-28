#ifndef BEAGLEPWM_H_
#define BEAGLEPWM_H_

#include "BeagleGPIO.h"
#include <stdint.h>


#define NUM_PWMS 3


class BeaglePWM
{

	public:
		BeaglePWM();
		virtual ~BeaglePWM();

		// use singleton?
		static BeaglePWM *getInstance();

		enum PwmPin {
			P8_13                   = 0,
			P8_19                   = 1,
			P9_14                   = 2,
			P9_16                   = 3,
			P9_21                   = 4,
			P9_22                   = 5,

			EHRPWM2B                = P8_13,
			EHRPWM2A                = P8_19,
			EHRPWM1A                = P9_14,
			EHRPWM1B                = P9_16,
			EHRPWM0B                = P9_21,
			EHRPWM0A                = P9_22
		};

		enum PwmModul {
			EHRPWM0	= 0,
			EHRPWM1 = 1,
			EHRPWM2 = 2
		};

		int setDutyCycle(PwmPin pin, unsigned long ns);
		int setRunState(PwmPin pin, bool enable);
		int setPeriod(PwmPin pin, unsigned long ns);

	private:
		struct PWMInfo
		{
				char *name;
				int pwmChip;
				int pwmPin;
		};
		int memFd; /* file descriptor for /dev/mem */
		uint16_t *pwmRegs[NUM_PWMS]; /** 3 mmaped pwm registers */
		uint16_t *ctrlRegs;
		uint16_t *clkRegs;
		uint16_t pwmPrescale[NUM_PWMS];
		uint16_t pwmPeriod[NUM_PWMS];
		static struct PWMInfo pwmInfos[];

		int setPrescaleDiv(PwmModul modul, uint32_t rqst_div);
		int modifyReg(uint16_t *reg, uint16_t address, uint16_t mask, uint16_t value);	// TODO Phileas fragen, ob das so möglich ist
};

#endif /* BEAGLEPWM_H_ */
