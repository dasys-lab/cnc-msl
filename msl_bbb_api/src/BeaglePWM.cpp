#include "BeaglePWM.h"
#include "debug.h"

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>


#define PWM_MMAPLEN 1024

// align for mmap pagesize
#define PWM_ADR_OFFSET	512


#define BIT(n) (1 << n)


/* Time base module registers */
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
const uint32_t BeaglePWM::pwmAddr[] = { 0x48300200, 0x48302200, 0x48304200 };

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
		pwmRegs[i] = (uint16_t *) mmap(NULL, PWM_MMAPLEN,
			PROT_READ | PROT_WRITE, MAP_SHARED, memFd, pwmAddr[i] - PWM_ADR_OFFSET);

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
	active = false;
	for (int i = 0; i < 4; i++)
		munmap(pwmRegs[i], PWM_MMAPLEN);
	close(memFd);
}

class BeaglePWM* BeaglePWM::getInstance()
{
	static BeaglePWM instance;
	return (BeaglePWM*) &instance;
}


int BeaglePWM::setDutyCycle(PwmPin pin, unsigned long us)
{
	// Berechnung der Periodenzyklen
	unsigned long duty_cycles = ns * 0.1;

	duty_cycles = duty_cycles / pwmPrescale;

	unsigned short modul = pwmInfos[pin].pwmChip;
	unsigned short cmp = (pwmInfos[pin].pwmPin ? CMPB : CMPA);
	
	pwmRegs[modul][(PWM_ADR_OFFSET + cmp) / 2] = duty_cycles;

	return 0;
}

int BeaglePWM::setRunState(PwmModul modul, bool enable)
{
	/* Changes to shadow mode */
	//ehrpwm_modify(pc->mmio_base, AQSFRC, AQSFRC_RLDCSF_MASK, AQSFRC_RLDCSF_ZRO);
	//ehrpwm_modify(pc->mmio_base, AQCSFRC, aqcsfrc_mask, aqcsfrc_val);

	/* Enable TBCLK before enabling PWM device */
	//ret = clk_enable(pc->tbclk);

	/* Enable time counter for free_run */
	modifyReg(pwmRegs[modul][(PWM_ADR_OFFSET + TBCTL) / 2], TBCTL, TBCTL_RUN_MASK, TBCTL_FREE_RUN);

	return -1;
}

int BeaglePWM::setPeriod(PwmModul modul, unsigned long ns)
{
	// TODO: Überprüfung ob in Gültigkeitsbereich

	// Berechnung der Periodenzyklen
	unsigned long period_cycles = ns * 0.1;

	setPrescaleDiv(modul, period_cycles/PERIOD_MAX);

	period_cycles = period_cycles / pwmPrescale;

	modifyReg(pwmRegs[modul][(PWM_ADR_OFFSET + TBCTL) / 2], TBCTL, TBCTL_PRDLD_MASK, TBCTL_PRDLD_SHDW);

	// TBPRD schreiben
	pwmRegs[modul][(PWM_ADR_OFFSET + TBPRD) / 2] = period_cycles;

	// TBCTL UpCountMode
	modifyReg(pwmRegs[modul][(PWM_ADR_OFFSET + TBCTL) / 2], TBCTL, TBCTL_CTRMODE_MASK, TBCTL_CTRMODE_UP);

	// TODO: DutyCycle neu setzen?
	// ggf einfach auf 0 setzen


	// Prescaler
		// set CLKDIV & HSCLKDIV

	// update periodcycles with new prescaler

	// shadow loading on period register

	// set TBPRD

	// set CTR Mode (up, down, updown, freeze)

	// set cmp

	return 0;
}

int BeaglePWM::setPrescaleDiv(PwmModul modul, uint32_t rqst_div)
{
	uint16_t clkdiv, hsclkdiv, prescale;

	for (clkdiv = 0; clkdiv++; clkdiv <= CLKDIV_MAX)
	{
		for (hsclkdiv = 0; hsclkdiv++; hsclkdiv <= HSPCLKDIV_MAX)
		{
			prescale = (1 << clkdiv) * (hsclkdiv ? (hsclkdiv * 2) : 1);
			if (prescale > rqst_div)
			{
				pwmPrescale[modul] = prescale;
				// TODO Phileas fragen
				modifyReg(pwmRegs[modul][(PWM_ADR_OFFSET + TBCTL) / 2], TBCTL, TBCTL_CLKDIV_MASK, TBCTL_CLKDIV_SHIFT | TBCTL_HSPCLKDIV_SHIFT);
//				uint16_t tbctl = pwmRegs[modul][(PWM_ADR_OFFSET + TBCTL) / 2] & ~(TBCTL_CLKDIV_MASK);
//				tbctl |= (clkdiv << TBCTL_CLKDIV_SHIFT) | (hsclkdiv << TBCTL_HSPCLKDIV_SHIFT);
//				pwmRegs[modul][(PWM_ADR_OFFSET + TBCTL) / 2] = tbctl;
				return 0;
			}
		}
	}

	return -1;
}

int BeaglePWM::modifyReg(unsigned short &reg, unsigned short address, unsigned short mask, unsigned short value)
{
	uint16_t tempreg;
	tempreg = reg & ~(mask);
	tempreg |= value;
	reg = tempreg;

	return 0;
}

