/*
 * ballhandle.h
 *
 *  Created on: Mar 11, 2015
 *      Author: Lukas Will
 */

#ifndef CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_BALLHANDLE_H_
#define CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_BALLHANDLE_H_
#define TIMEDIFFMS(n,o) (((n).tv_sec-(o).tv_sec)*1000+((n).tv_usec-(o).tv_usec)/1000)
#define BallHandle_TIMEOUT 1000

#include <sys/time.h>

#include "BlackDef.h"
#include "BlackGPIO.h"
#include "BlackPWM.h"



class BallHandle {
	private:
		BlackLib::BlackPWM	*pwm;
		BlackLib::BlackGPIO	*dir, *reset, *ff1, *ff2;

		BlackLib::digitalValue	direction			= static_cast<BlackLib::digitalValue>(left);
		BlackLib::digitalValue	direction_desired	= static_cast<BlackLib::digitalValue>(left);

		bool			enabled = false;
		const int		period = 10000;
		int				speed = 0, speed_desired = 0;
		timeval			last_ping;

	public:
		enum errorList {
			none			= 0,
			bypass			= 1,
			temperature		= 2,
			voltage			= 3
		};

		enum directionList {
			left			= 0,
			right			= 1
		};


				BallHandle(BlackLib::pwmName pwm_P, BlackLib::gpioName dir_P, BlackLib::gpioName reset_P, BlackLib::gpioName ff1_P, BlackLib::gpioName ff2_P);
				~BallHandle();


		void	setBallHandling(int8_t value);
		void	checkTimeout();
		void	controlBallHandling();

		int		getError();
};


#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_BALLHANDLE_H_ */
