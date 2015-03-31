/*
 * opticalflow.h
 *
 *  Created on: Mar 26, 2015
 *      Author: Lukas Will
 */

#ifndef CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_OPTICALFLOW_H_
#define CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_OPTICALFLOW_H_


#include "config.h"
#include "BlackDef.h"
#include "BlackGPIO.h"


class OpticalFlow {
	private:
		BlackLib::BlackGPIO *ncs, *npd, *rst, *led;



	public:
		OpticalFlow(BlackLib::gpioName ncs_P, BlackLib::gpioName npd_P, BlackLib::gpioName rst_P, BlackLib::gpioName led_P);
		~OpticalFlow();


};



#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_OPTICALFLOW_H_ */
