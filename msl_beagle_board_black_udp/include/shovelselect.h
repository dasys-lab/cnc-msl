/*
 * shovelselect.h
 *
 *  Created on: Mar 7, 2016
 *      Author: Lukas Will
 */

#ifndef INCLUDE_SHOVELSELECT_H_
#define INCLUDE_SHOVELSELECT_H_
#define TIMEDIFFMS(n,o) (((n).tv_sec-(o).tv_sec)*1000+((n).tv_usec-(o).tv_usec)/1000)

#include <sys/time.h>

#include "BlackDef.h"
#include "BlackPWM.h"


class ShovelSelect {
	private:
		BlackLib::BlackPWM	*pwm;
		const int	period = 20000000;

		bool		enabled, init, statePassing;
		int			kickPWM = 1600000, passPWM = 1300000;
		int			timeout;
		timeval		ping;


	public:
				ShovelSelect(BlackLib::pwmName pwm_P);
				~ShovelSelect();

		bool	checkTimeout(timeval time);
		bool	setShovel(bool passing, timeval time_now);
};



#endif /* INCLUDE_SHOVELSELECT_H_ */
