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
#include <BeaglePWM.h>

class ShovelSelect
{
private:
	BeaglePWM *pwm;
	BeaglePWM::PwmPin pwm_pin;

	bool enabled;
	bool init;
	bool statePassing;
	const int period = 20000000;
	int kickPWM = 1600000;
	int passPWM = 1300000;
	int timeout;
	timeval ping;

public:
	ShovelSelect(BeaglePWM::PwmPin pwm_name);
	~ShovelSelect();

	bool checkTimeout(timeval time);
	bool setShovel(bool passing, timeval time_now);
};

#endif /* INCLUDE_SHOVELSELECT_H_ */
