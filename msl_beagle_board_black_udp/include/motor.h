#ifndef CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_MOTOR_H_
#define CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_MOTOR_H_

#include <BeagleGPIO.h>
#include <BeaglePins.h>
#include <BeaglePWM.h>

enum Error
{
	none, bypass, temperature, voltage, programming
};

enum Direction
{
	left = 0, right = 1
};

enum BH_Pin
{
	dir, rst, ff1, ff2
};

class Motor
{
public:
	Motor(BeaglePWM::PwmPin pwm_name, const char *pin_names[], int period);
	~Motor();

	void setSpeed(int speed);

	Error getError();

private:
	BeagleGPIO *gpio;
	BeaglePins *pins;
	BeaglePWM *pwm;
	BeaglePWM::PwmPin pwm_pin;

	int maxSpeed;

	Direction direction;

	void setDirection(Direction desiredDirection);
};


#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_INCLUDE_MOTOR_H_ */
