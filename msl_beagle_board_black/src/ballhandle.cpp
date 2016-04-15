/*
 * ballhandle.cpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Lukas Will
 */


#include "ballhandle.h"

using namespace BlackLib;

	BallHandle::BallHandle(pwmName pwm_P, gpioName dir_P, gpioName reset_P, gpioName ff1_P, gpioName ff2_P) {
		pwm = new BlackPWM(pwm_P);
		dir = new BlackGPIO(dir_P, output, FastMode);
		reset = new BlackGPIO(reset_P, output, FastMode);
		ff1 = new BlackGPIO(ff1_P, input, FastMode);
		ff2 = new BlackGPIO(ff2_P, input, FastMode);


		pwm->setPeriodTime(period, nanosecond);
		pwm->setSpaceRatioTime(0, nanosecond);
		pwm->setRunState(run);

		dir->setValue(low);
		reset->setValue(high);
	}

	BallHandle::~BallHandle() {
		delete pwm;
		delete dir;
		delete reset;
		delete ff1;
		delete ff2;
	}

	void BallHandle::setBallHandling(int8_t value) {
		// value > 0 -> left
		// value < 0 -> right
		if ((value > 0) && (direction == static_cast<digitalValue>(right))) {
			direction_desired = static_cast<digitalValue>(left);
		}

		if ((value < 0) && (direction == static_cast<BlackLib::digitalValue>(left))) {
			direction_desired = static_cast<BlackLib::digitalValue>(right);
		}

		// Check that value is in range from -period to period
		if (value > period) { value = period; }
		if (value < -period) { value = -period; }
		speed_desired = abs(value);

		gettimeofday(&last_ping, NULL);
	}

	void BallHandle::checkTimeout() {
		// Deactivates the BallHandling when controlBallHandling() is called next time
		timeval	t;
		gettimeofday(&t, NULL);
		if (TIMEDIFFMS(t, last_ping) > BallHandle_TIMEOUT) {
			this->setBallHandling(0);
		}
	}

	void BallHandle::controlBallHandling() {
		checkTimeout();
		if (direction != direction_desired && speed_desired != 0) {
			direction = direction_desired;
			speed = 0;
			pwm->setSpaceRatioTime(speed, nanosecond);		// Time for this Operation: 900us
			dir->setValue(direction);						// Time for this Operation: 550us
		}

		if (speed != speed_desired) {
			speed = speed_desired;
			pwm->setSpaceRatioTime(speed, nanosecond);		// Time for this Operation: 900us
		}
	}

	int BallHandle::getError() {
		int ff = (ff1->getNumericValue() << 1) | ff2->getNumericValue();

		return ff;
	}


