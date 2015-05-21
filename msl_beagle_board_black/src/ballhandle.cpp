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

		// PWM Frequenz setzen pwm->setPeriodTime(5000, microsecond);
		pwm->setPeriodTime(10000, nanosecond);

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

		speed_desired = abs(value) * 39;
	}

	void BallHandle::setTimeout() {
		if (enabled) {
			this->setBallHandling(0);		// Beim naechsten Aufruf von controlBallHandling() wird das BallHandling deaktiviert
		}
	}

	void BallHandle::controlBallHandling() {
		if (speed_desired == 0) {
			enabled = false;

			if (pwm->getRunValue() == "1") {						// 300us
				pwm->setRunState(stop);								// ?us
			}
		} else if ((speed_desired != 0) && (!enabled)) {
			enabled = true;
			pwm->setRunState(run);
		}

		if (enabled) {	// Gesamt ca. 900us oder 1500us
			// BallHandling active

			if (direction != direction_desired) {
				// Direction Change 1500us
				direction = direction_desired;
				speed = 0;
				pwm->setSpaceRatioTime(speed, nanosecond);		// 900us
				dir->setValue(direction);						// 550us
			}

			if (speed != speed_desired) {
				// Speed Change 900us
				speed = speed_desired;
				pwm->setSpaceRatioTime(speed, nanosecond);		// 900us
			}

		}



		/* ALTE FUNKTION
		if (speed_desired == 0) {
			enabled = false;

			if (pwm->getRunValue() == "1") {						// 300us
				pwm->setRunState(stop);								// ?us
			}
		} else if ((speed_desired != 0) && (!enabled)) {
			enabled = true;
			pwm->setRunState(run);
		}

		if (enabled) {	// Gesamt ca. 900us oder 1500us
			// BallHandling aktiviert

			if (direction != direction_desired) {			// Direction Change, Slow Down Speed (Gesamt: 900us oder 1500us)
				speed -= BallHandle_PWM_STEP_SIZE;
				if (speed < 0) {
					direction = direction_desired;
					speed = 0;
					dir->setValue(direction);				// 550us
				}
			} else {										// Keep Direction, Modify Speed
				if (speed_desired > speed) {
					speed += BallHandle_PWM_STEP_SIZE;
					if (speed > speed_desired)
						speed = speed_desired;
				} else {
					speed -= BallHandle_PWM_STEP_SIZE;
					if (speed < speed_desired)
						speed = speed_desired;
				}
			}
			pwm->setSpaceRatioTime(speed, microsecond);			// 900us
		}
		*/

	}

	uint8_t BallHandle::getError() {
		uint8_t ff = (ff1->getNumericValue() << 1) | ff2->getNumericValue();

		return ff;
	}


