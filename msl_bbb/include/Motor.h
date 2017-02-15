#pragma once

#include <BeaglePWM.h>

#include <vector>

class BeagleGPIO;
class BeaglePins;
class BeaglePWM;
namespace msl_bbb
{

enum Error
{
    none,
    bypass,
    temperature,
    voltage,
    programming
};

enum Direction
{
    left = 0,
    right = 1
};

enum BH_Pin
{
    dir, // direction pin
    rst, // reset pin
    ff1, // failure pin 1
    ff2  // failure pin 2
};

class Motor
{
  public:
    Motor(BeaglePWM::PwmPin pwm_name, std::vector<char const *> pin_names);
    ~Motor();

    void setSpeed(int speed);
    Error readError();

  private:
    void setDirection(Direction desiredDirection);

    BeagleGPIO *gpio;
    BeaglePins *pins;
    BeaglePWM *pwm;
    BeaglePWM::PwmPin pwm_pin;

    const int maxSpeed = 10000;

    Direction direction;
};
}
