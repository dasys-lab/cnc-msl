#include "Motor.h"

#include <BeagleGPIO.h>
#include <BeaglePins.h>
#include <stdlib.h>

namespace msl_bbb
{

Motor::Motor(BeaglePWM::PwmPin pwm_name, std::vector<char const*> pin_names)
{
    gpio = BeagleGPIO::getInstance();
    pwm = BeaglePWM::getInstance();
    // NOTE: critical cast (hopefully claim does not change the chars)
    pins = gpio->claim((char**)pin_names.data(), 4);

    int outputIdxs[] = {dir, rst};
    pins->enableOutput(outputIdxs, 2);

    pins->clearBit(dir);
    pins->setBit(rst);

    pwm_pin = pwm_name;
    pwm->setPeriod(pwm_pin, this->maxSpeed);
    pwm->setRunState(pwm_pin, true);
    pwm->setDutyCycle(pwm_pin, 0);

    direction = left;
}

Motor::~Motor()
{
    pwm->setRunState(pwm_pin, false);
    // FIXME: delete object that is created with singleton?
    delete pwm;
    delete gpio;
}

/**
 * Public method for setting the motor speed.
 * It determines the direction according to the sign
 * of speed and set the pwm with abs(speed).
 * @param speed
 */
void Motor::setSpeed(int speed)
{
    if (speed > 0)
        setDirection(left);

    if (speed < 0)
        setDirection(right);

    int s = (abs(speed) < maxSpeed) ? abs(speed) : maxSpeed;

    pwm->setDutyCycle(pwm_pin, s);
}

/**
 * Sets the direction of the motor.
 * @param desiredDirection
 */
void Motor::setDirection(Direction desiredDirection)
{
    if (direction != desiredDirection)
    {
        direction = desiredDirection;
        pwm->setDutyCycle(pwm_pin, 0);
        if (direction)
            pins->setBit(dir);
        else
            pins->clearBit(dir);
    }
}

/**
 * Read the error state of the motor
 * controller.
 * @return error state
 */
Error Motor::readError()
{
    int pin1 = pins->getBit(ff1);
    int pin2 = pins->getBit(ff2);

    if (pin1 < 0 || pin2 < 0)
        return programming;
    if (!pin1 && pin2)
        return bypass;
    if (pin1 && !pin2)
        return temperature;
    if (pin1 && pin2)
        return voltage;

    //!pin1 && !pin2
    return none;
}
}
