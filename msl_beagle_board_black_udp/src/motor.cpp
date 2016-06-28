#include "motor.h"

#include <stdlib.h>

Motor::Motor(BeaglePWM::PwmPin pwm_name, const char *pin_names[], int period)
{
	gpio = BeagleGPIO::getInstance();
	pwm = BeaglePWM::getInstance();
	pins = gpio->claim((char**) pin_names, 4);

	int outputIdxs[] = { dir, rst};
	pins->enableOutput(outputIdxs, 2);

	pins->clearBit(dir);
	pins->setBit(rst);

	pwm_pin = pwm_name;
	pwm->setPeriod(pwm_pin, period);
	pwm->setRunState(pwm_pin, true);
	pwm->setDutyCycle(pwm_pin, 0);

	maxSpeed = period;
	direction = left;
}

Motor::~Motor()
{
	pwm->setRunState(pwm_pin, false);
	delete pwm;
	delete gpio;
}

void Motor::setSpeed(int speed)
{
	if (speed > 0)
		setDirection(left);

	if (speed < 0)
		setDirection(right);

	int s = (abs(speed) < maxSpeed) ? abs(speed) : maxSpeed;

	pwm->setDutyCycle(pwm_pin, s);
}

void Motor::setDirection(Direction desiredDirection)
{
	if (direction != desiredDirection) {
		direction = desiredDirection;
		pwm->setDutyCycle(pwm_pin, 0);
		if (direction)
			pins->setBit(dir);
		else
			pins->clearBit(dir);
	}
}

Error Motor::getError()
{
	int pin1 = pins->getBit(ff1);
	int pin2 = pins->getBit(ff2);

	if (pin1 < 0 || pin2 < 0)
		return programming;

	if (!pin1 && !pin2)
		return none;
	if (!pin1 && pin2)
		return bypass;
	if (pin1 && !pin2)
		return temperature;
	if (pin1 && pin2)
		return voltage;
}
