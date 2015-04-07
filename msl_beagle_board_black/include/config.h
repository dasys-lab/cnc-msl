/*
 * config.h
 *
 *  Created on: Mar 12, 2015
 *      Author: Lukas Will
 */

#ifndef CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_CONFIG_H_
#define CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_CONFIG_H_

#include <iostream>
#include <sstream>
#include <stdint.h>
#include <sys/time.h>

#define TIMEDIFFUS(n,o) (((n).tv_usec-(o).tv_usec))
#define TIMEDIFFMS(n,o) (((n).tv_sec-(o).tv_sec)*1000+((n).tv_usec-(o).tv_usec)/1000)

#define PING_TIMEOUT				1000

#define BallHandle_TIMEOUT			1000	// ms
#define BallHandle_PWM_STEP_SIZE	50

#define ShovelSelect_TIMEOUT		1000	// ms
#define ShovelSelect_PASSING		1000	// PWM ( 1ms / 20ms )
#define ShovelSelect_NORMAL			2000	// PWM ( 2ms / 20ms )


// Entscheidungsschwelle fuer Ball (0 bis 65000)
const uint16_t LIGHTBARRIER_THRESHOLD = 2500;

struct Shovel {
	bool		enabled;
	uint16_t	value;
	timeval		last_ping;
};



#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_CONFIG_H_ */
