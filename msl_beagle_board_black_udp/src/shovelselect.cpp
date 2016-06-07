/*
 * shovelselect.cpp
 *
 *  Created on: Mar 7, 2016
 *      Author: Lukas Will
 */

#include "shovelselect.h"
#include <SystemConfig.h>

	ShovelSelect::ShovelSelect(BeaglePWM::PwmPin pwm_name) {
		pwm = BeaglePWM::getInstance();

		pwm_pin = pwm_name;
		pwm->setPeriod(pwm_pin, period);
		pwm->setRunState(pwm_pin, true);
		pwm->setDutyCycle(pwm_pin, 0);

		auto sc = supplementary::SystemConfig::getInstance();
		this->kickPWM = (*sc)["bbb"]->get<int>("BBB.shovelKick", NULL);
		this->passPWM = (*sc)["bbb"]->get<int>("BBB.shovelPass", NULL);
		this->timeout = (*sc)["bbb"]->get<int>("BBB.timeout", NULL);
std::cout << "INIT: " << kickPWM << " - " << passPWM << std::endl;		
		enabled = false;
		init = false;
	}

	ShovelSelect::~ShovelSelect() {
		pwm->setRunState(pwm_pin, false);
		delete pwm;
	}

	bool ShovelSelect::checkTimeout(timeval time) {
		if ((TIMEDIFFMS(time, ping) > timeout) && enabled) {
			pwm->setDutyCycle(pwm_pin, 0);
			enabled = false;

			return true;
		}

		return false;
	}

	bool ShovelSelect::setShovel(bool passing, timeval time_now) {
		if (statePassing == passing && init) {
std::cout << "1" << std::endl;
			return false;
		}

std::cout << "setShovel: " << passing << std::endl;		
		init = true;
		ping = time_now;
		statePassing = passing;
		if (passing) {
std::cout << "pass " << passPWM * 1000 << std::endl;
			pwm->setDutyCycle(pwm_pin, passPWM * 1000);	// * 1000 because ns needed and passPWM is in us
		} else {
std::cout << "kick " << kickPWM * 1000 << std::endl;
			pwm->setDutyCycle(pwm_pin, kickPWM * 1000);	// * 1000 because ns needed and kickPWM is in us
		}
		if (!enabled) {
std::cout << "en" << std::endl;
			enabled = true;
		}

std::cout << "2" << std::endl;
		return true;
	}

