/*
 * ballhandling.h
 *
 *  Created on: Mar 11, 2015
 *      Author: Lukas Will
 */

#ifndef CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_BALLHANDLING_H_
#define CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_BALLHANDLING_H_

#include <stdint.h>

#include "BlackGPIO.h"
#include "BlackPWM.h"

enum errorList {
	none			= 0,
	bypass			= 1,
	temperature		= 2,
	voltage			= 3
};

uint8 getError(BlackLib::BlackGPIO &ff1, BlackLib::BlackGPIO &ff2);

#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_BALLHANDLING_H_ */
