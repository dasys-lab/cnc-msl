#include "BeaglePWM.h"
#include "debug.h"

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

/*
   addresses for epwmss0, epwmss1, epwmss2 
   see: your device tree file
*/
const uint32_t BeaglePWM::pwmAddr[] = { 0x48300200, 0x48302200, 0x48304200 };

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
		pwmRegs[i] = (uint32_t *) mmap(NULL, PWM_REGLEN,
			PROT_READ | PROT_WRITE, MAP_SHARED, memFd, gpioAddrs[i]);
	}

	if (gpios[i] == MAP_FAILED )
	{
		debug(0, "PWM Mapping failed for PWM Module %i\n", i);
		return;
	}
}

//Destructor
BeagleGPIO::~BeagleGPIO()
{
	active = false;
	for (int i = 0; i < 4; i++)
		munmap(gpios[i], GpioMemBlockLength);
	close(gpioFd);
}


void BeagleGPIO::setDutyCycle(int pin, unsigned long us)
{
	
}
 
class BeagleGPIO* BeagleGPIO::getInstance()
{
	static BeagleGPIO instance;
	return (BeagleGPIO*) &instance;
}

