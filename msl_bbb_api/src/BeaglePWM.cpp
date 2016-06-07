#include "BeaglePWM.h"
#include "debug.h"

#include <stdio.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define PWM_MMAPLEN 1024

// align for mmap pagesize
#define PWM_ADR_OFFSET	512


#define BIT(n) (1 << n)


/* Time base module registers and masks */
#define TBCTL			0x00
#define TBPRD			0x0A

#define TBCTL_RUN_MASK		(BIT(15) | BIT(14))
#define TBCTL_STOP_NEXT		0
#define TBCTL_STOP_ON_CYCLE	BIT(14)
#define TBCTL_FREE_RUN		(BIT(15) | BIT(14))
#define TBCTL_PRDLD_MASK	BIT(3)
#define TBCTL_PRDLD_SHDW	0
#define TBCTL_PRDLD_IMDT	BIT(3)
#define TBCTL_CLKDIV_MASK	(BIT(12) | BIT(11) | BIT(10) | BIT(9) | BIT(8) | BIT(7))
#define TBCTL_CTRMODE_MASK	(BIT(1) | BIT(0))
#define TBCTL_CTRMODE_UP	0
#define TBCTL_CTRMODE_DOWN	BIT(0)
#define TBCTL_CTRMODE_UPDOWN	BIT(1)
#define TBCTL_CTRMODE_FREEZE	(BIT(1) | BIT(0))

#define TBCTL_HSPCLKDIV_SHIFT	7
#define TBCTL_CLKDIV_SHIFT		10

#define CLKDIV_MAX		7
#define HSPCLKDIV_MAX	7
#define PERIOD_MAX		0xFFFF

/* compare module registers */
#define CMPA			0x12
#define CMPB			0x14

/* Action qualifier module registers */
#define AQCTLA			0x16
#define AQCTLB			0x18
#define AQSFRC			0x1A
#define AQCSFRC			0x1C

#define AQCTL_CBU_MASK		(BIT(9) | BIT(8))
#define AQCTL_CBU_FRCLOW	BIT(8)
#define AQCTL_CBU_FRCHIGH	BIT(9)
#define AQCTL_CBU_FRCTOGGLE	(BIT(9) | BIT(8))
#define AQCTL_CAU_MASK		(BIT(5) | BIT(4))
#define AQCTL_CAU_FRCLOW	BIT(4)
#define AQCTL_CAU_FRCHIGH	BIT(5)
#define AQCTL_CAU_FRCTOGGLE	(BIT(5) | BIT(4))
#define AQCTL_PRD_MASK		(BIT(3) | BIT(2))
#define AQCTL_PRD_FRCLOW	BIT(2)
#define AQCTL_PRD_FRCHIGH	BIT(3)
#define AQCTL_PRD_FRCTOGGLE	(BIT(3) | BIT(2))
#define AQCTL_ZRO_MASK		(BIT(1) | BIT(0))
#define AQCTL_ZRO_FRCLOW	BIT(0)
#define AQCTL_ZRO_FRCHIGH	BIT(1)
#define AQCTL_ZRO_FRCTOGGLE	(BIT(1) | BIT(0))

#define AQCTL_CHANA_POLNORMAL	(AQCTL_CAU_FRCLOW | AQCTL_PRD_FRCHIGH | AQCTL_ZRO_FRCHIGH)
#define AQCTL_CHANA_POLINVERSED	(AQCTL_CAU_FRCHIGH | AQCTL_PRD_FRCLOW | AQCTL_ZRO_FRCLOW)
#define AQCTL_CHANB_POLNORMAL	(AQCTL_CBU_FRCLOW | AQCTL_PRD_FRCHIGH | AQCTL_ZRO_FRCHIGH)
#define AQCTL_CHANB_POLINVERSED	(AQCTL_CBU_FRCHIGH | AQCTL_PRD_FRCLOW | AQCTL_ZRO_FRCLOW)

#define AQSFRC_RLDCSF_MASK	(BIT(7) | BIT(6))
#define AQSFRC_RLDCSF_ZRO	0
#define AQSFRC_RLDCSF_PRD	BIT(6)
#define AQSFRC_RLDCSF_ZROPRD	BIT(7)
#define AQSFRC_RLDCSF_IMDT	(BIT(7) | BIT(6))

#define AQCSFRC_CSFB_MASK	(BIT(3) | BIT(2))
#define AQCSFRC_CSFB_FRCDIS	0
#define AQCSFRC_CSFB_FRCLOW	BIT(2)
#define AQCSFRC_CSFB_FRCHIGH	BIT(3)
#define AQCSFRC_CSFB_DISSWFRC	(BIT(3) | BIT(2))
#define AQCSFRC_CSFA_MASK	(BIT(1) | BIT(0))
#define AQCSFRC_CSFA_FRCDIS	0
#define AQCSFRC_CSFA_FRCLOW	BIT(0)
#define AQCSFRC_CSFA_FRCHIGH	BIT(1)
#define AQCSFRC_CSFA_DISSWFRC	(BIT(1) | BIT(0))

#define NUM_PWM_CHANNEL		2	/* EHRPWM channels */


/*
   addresses for epwmss0, epwmss1, epwmss2 
   see: your device tree file
*/
const uint32_t pwmAddr[] = { 0x48300000, 0x48302000, 0x48304000 };

struct BeaglePWM::PWMInfo BeaglePWM::pwmInfos[] =
{
 	{(char*) "P8_13", 2, 1},
	{(char*) "P8_19", 2, 0},
	{(char*) "P9_14", 1, 0},
	{(char*) "P9_16", 1, 1},
	{(char*) "P9_21", 0, 1},
	{(char*) "P9_22", 0, 0},
};

//Constructor		
BeaglePWM::BeaglePWM()
{
	//Open File Device /dev/mem
	memFd = open("/dev/mem", O_RDWR | O_SYNC);
	if (memFd < 0)
	{
		debug(0, "BeaglePWM::BeaglePWM(): Can't open /dev/mem\n");
		return;
	}

	//PWM mapping
	for (int i = 0; i < NUM_PWMS; i++) {
		pwmRegs[i] = (uint16_t *) mmap(NULL, PWM_MMAPLEN, PROT_READ | PROT_WRITE, MAP_SHARED, memFd, pwmAddr[i]);

		if (pwmRegs[i] == MAP_FAILED )
		{
			debug(0, "PWM Mapping failed for PWM Module %i\n", i);
			return;
		}
	}
}

//Destructor
BeaglePWM::~BeaglePWM()
{
	for (int i = 0; i < 4; i++)
		munmap(pwmRegs[i], PWM_MMAPLEN);
	close(memFd);
}

class BeaglePWM* BeaglePWM::getInstance()
{
	static BeaglePWM instance;
	return (BeaglePWM*) &instance;
}


int BeaglePWM::setDutyCycle(PwmPin pin, unsigned long ns)
{
	unsigned short modul = pwmInfos[pin].pwmChip;
	unsigned short cmp;

	pwmRegs[modul][(PWM_ADR_OFFSET + TBPRD) / 2] = pwmPeriod[modul];

	// Calculate cycles
	unsigned long duty_cycles = ns * 0.1;
	duty_cycles = duty_cycles / pwmPrescale[modul];
	if (pwmInfos[pin].pwmPin == 0)
	{
		pwmRegs[modul][(PWM_ADR_OFFSET + CMPA) / 2] = duty_cycles;
	}
	else
	{
		pwmRegs[modul][(PWM_ADR_OFFSET + CMPB) / 2] = duty_cycles;
	}

	return 0;
}

int BeaglePWM::setRunState(PwmPin pin, bool enable)
{
	uint16_t aqcsfrc, aqcsfrc_mask, aqcsfrc_val;
	uint16_t aqctl, aqctl_mask, aqctl_val;
	int modul = pwmInfos[pin].pwmChip;

	if (enable)
	{
		if (pwmInfos[pin].pwmPin == 0)
		{
			aqcsfrc = AQCSFRC_CSFA_MASK;
			aqcsfrc_mask = AQCSFRC_CSFA_MASK;
			aqcsfrc_val = AQCSFRC_CSFA_FRCDIS;
			aqctl = AQCTLA;
			aqctl_mask = AQCTL_CAU_MASK;
			aqctl_val = AQCTL_CHANA_POLNORMAL;
		}
		else
		{
			aqcsfrc = AQCSFRC_CSFB_MASK;
			aqcsfrc_mask = AQCSFRC_CSFB_MASK;
			aqcsfrc_val = AQCSFRC_CSFB_FRCDIS;
			aqctl = AQCTLB;
			aqctl_mask = AQCTL_CBU_MASK;
			aqctl_val = AQCTL_CHANB_POLNORMAL;
		}
		// Configure Action-Qualifier - Software Force Register
		modifyReg(pwmRegs[modul], AQSFRC, AQSFRC_RLDCSF_MASK, AQSFRC_RLDCSF_ZRO);
		modifyReg(pwmRegs[modul], aqcsfrc, aqcsfrc_mask, aqcsfrc_val);

		// Configure Action-Qualifier Control Register (TODO: change this, if you want to implement polarity)
		aqctl_mask |= AQCTL_PRD_MASK | AQCTL_ZRO_MASK;
		modifyReg(pwmRegs[modul], aqctl, aqctl_mask, aqctl_val);

		/* Enable time counter for free_run */
		modifyReg(pwmRegs[modul], TBCTL, TBCTL_RUN_MASK, TBCTL_FREE_RUN);
	} else
	{
		if (pwmInfos[pin].pwmPin == 0)
		{
			aqcsfrc = AQCSFRC_CSFA_MASK;
			aqcsfrc_mask = AQCSFRC_CSFA_MASK;
			aqcsfrc_val = AQCSFRC_CSFA_FRCLOW;
		}
		else
		{
			aqcsfrc = AQCSFRC_CSFB_MASK;
			aqcsfrc_mask = AQCSFRC_CSFB_MASK;
			aqcsfrc_val = AQCSFRC_CSFA_FRCLOW;
		}


		/*
		 * Changes to immediate action on Action Qualifier. This puts
		 * Action Qualifier control on PWM output from next TBCLK
		 */
		modifyReg(pwmRegs[modul], AQSFRC, AQSFRC_RLDCSF_MASK,AQSFRC_RLDCSF_IMDT);
		modifyReg(pwmRegs[modul], AQCSFRC, aqcsfrc_mask, aqcsfrc_val);

		setDutyCycle(pin, 0);

		/* Stop Time base counter */
		modifyReg(pwmRegs[modul], TBCTL, TBCTL_RUN_MASK, TBCTL_STOP_NEXT);
	}

	return 0;
}

int BeaglePWM::setPeriod(PwmPin pin, unsigned long ns)
{
	// TODO: Überprüfung ob in Gültigkeitsbereich
	// Duty_cycle darf nicht größer als period sein

	PwmModul modul = (PwmModul) pwmInfos[pin].pwmChip;

	// Berechnung der Periodenzyklen
	unsigned long period_cycles = ns * 0.1;
	setPrescaleDiv(modul, period_cycles/PERIOD_MAX);
	pwmPeriod[modul] = period_cycles / pwmPrescale[modul];

	modifyReg(pwmRegs[modul], TBCTL, TBCTL_PRDLD_MASK, TBCTL_PRDLD_SHDW);
	pwmRegs[modul][(PWM_ADR_OFFSET + TBPRD) / 2] = pwmPeriod[modul];
	modifyReg(pwmRegs[modul], TBCTL, TBCTL_CTRMODE_MASK, TBCTL_CTRMODE_UP);

	// TODO: DutyCycle neu setzen?
	// ggf einfach auf 0 setzen

	return 0;
}

int BeaglePWM::setPrescaleDiv(PwmModul modul, uint32_t rqst_div)
{
	uint16_t clkdiv, hsclkdiv, prescale;

	for (clkdiv = 0; clkdiv <= CLKDIV_MAX; clkdiv++)
	{
		for (hsclkdiv = 0; hsclkdiv <= HSPCLKDIV_MAX; hsclkdiv++)
		{
			prescale = (1 << clkdiv) * (hsclkdiv ? (hsclkdiv * 2) : 1);
			if (prescale > rqst_div)
			{
				pwmPrescale[modul] = prescale;
				modifyReg(pwmRegs[modul], TBCTL, TBCTL_CLKDIV_MASK, TBCTL_CLKDIV_SHIFT | TBCTL_HSPCLKDIV_SHIFT);
				return 0;
			}
		}
	}

	return -1;
}

int BeaglePWM::modifyReg(uint16_t *reg, uint16_t address, uint16_t mask, uint16_t value)
{
	uint16_t tempreg = reg[(PWM_ADR_OFFSET + address) / 2];
	tempreg &= ~mask;
	tempreg |= value & mask;
	reg[(PWM_ADR_OFFSET + address) / 2] = tempreg;

	return 0;
}

