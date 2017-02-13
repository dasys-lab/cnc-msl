#pragma once

#include "Worker.h"

#include <ros/ros.h>

#include <thread>
#include <vector>

namespace msl_actuator_msgs{
	ROS_DECLARE_MESSAGE(BallHandleCmd);
	ROS_DECLARE_MESSAGE(BallHandleMode);
}

namespace msl_bbb{

class Motor;

class BallHandler : public Worker
{
public:
	BallHandler();
	~BallHandler();

	void run(); /** < overwrites the workers virtual run method */

	void dribbleControl();
	void setBallHandling(int32_t left, int32_t right);

	void handleBallHandleControl(const msl_actuator_msgs::BallHandleCmd msg);
	void handleBallHandleMode(const msl_actuator_msgs::BallHandleMode msg);

	bool checkTimeout();
	void updateTimeOfLastCmd();

private:

	std::vector<char const *>BH_right_pins;
	std::vector<char const *>BH_left_pins;

	Motor *rightMotor;
	Motor *leftMotor;
	uint8_t mode; /** < control mode (autonomous or remote) */

	int timeout; /** < stop motors after timeout happened */
	timeval timeOfLastCommand; /** < time of last remote command */
};

}
