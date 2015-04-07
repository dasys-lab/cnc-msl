/*
 * opticalflow.cpp
 *
 *  Created on: Mar 26, 2015
 *      Author: Lukas Will
 */

#include "opticalflow.h"

using namespace BlackLib;

OpticalFlow::OpticalFlow(gpioName ncs_P, gpioName npd_P, gpioName rst_P, gpioName led_P) {
	ncs = new BlackGPIO(ncs_P, output, FastMode);
	npd = new BlackGPIO(npd_P, output, FastMode);
	rst = new BlackGPIO(rst_P, output, FastMode);
	led = new BlackGPIO(led_P, output, FastMode);

	/* BlackLib::BlackGPIO OF_NPD(GPIO_117, output, FastMode);	// P8 07
	 * BlackLib::BlackGPIO OF_RST(GPIO_115, output, FastMode);	// P8 07
	 * BlackLib::BlackGPIO OF_NCS(GPIO_112, output, FastMode);	// P8 07 */

}

OpticalFlow::~OpticalFlow() {
	delete ncs;
	delete npd;
	delete rst;
	delete led;
}
