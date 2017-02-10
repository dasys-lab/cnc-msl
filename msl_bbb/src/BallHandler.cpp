#include "BallHandler.h"

#include "Motor.h"

#include <msl_actuator_msgs/BallHandleCmd.h>
#include <msl_actuator_msgs/BallHandleMode.h>

#include <BeaglePWM.h>
#include <SystemConfig.h>

#include <math.h>
#include <stdlib.h>
#include <sys/time.h>

namespace msl_bbb
{

BallHandler::BallHandler()
    : Worker("BallHandler")
{
    BH_right_pins = {"P8_9", "P8_10", "P8_16", "P8_18"};
    BH_left_pins = {"P8_7", "P8_8", "P8_12", "P8_14"};

    rightMotor = new Motor(BeaglePWM::P8_19, BH_right_pins);
    leftMotor = new Motor(BeaglePWM::P8_13, BH_left_pins);

    // remote control is default mode
    this->mode = msl_actuator_msgs::BallHandleMode::REMOTE_CONTROL;

    auto sc = supplementary::SystemConfig::getInstance();
    this->timeout = (*sc)["bbb"]->get<int>("BBB.timeout", NULL);
}

BallHandler::~BallHandler()
{
    delete rightMotor;
    delete leftMotor;
}

/**
 * Run method of executed through the Worker base class.
 */
void BallHandler::run()
{
    switch (this->mode)
    {
    case msl_actuator_msgs::BallHandleMode::REMOTE_CONTROL:
        this->checkTimeout();
        break;
    case msl_actuator_msgs::BallHandleMode::AUTONOMOUS_CONTROL:
        this->dribbleControl();
        break;
    }
}

/**
 * This method is doing the ball handle control in
 * autonomous control mode.
 *
 * TODO: Currently its empty.
 */
void BallHandler::dribbleControl()
{
    std::cerr << "BallHandler: dribbleControl() not implemented, yet!" << std::endl;
}

/**
 * Sets the left and right motor speed.
 * @param speedL speed of left motor
 * @param speedR speed of right motor
 */
void BallHandler::setBallHandling(int32_t speedL, int32_t speedR)
{
    leftMotor->setSpeed(speedL);
    rightMotor->setSpeed(speedR);
    cout << "BallHandler: Mode(REMOTE):  Left: " << speedL << ",\t Right: " << speedR << endl;
}

/**
 * Sets current time to timeOfLastCommand. This method is called
 * by every ROS msg receiving method.
 */
void BallHandler::updateTimeOfLastCmd()
{
    gettimeofday(&timeOfLastCommand, NULL);
}

/**
 * Checks when the last time updateTimeOfLastCmd() was called and
 * sets the motor speeds to 0, if the timeout happened.
 */
bool BallHandler::checkTimeout()
{
    timeval t;
    gettimeofday(&t, NULL);
    if (TIMEDIFFMS(t, timeOfLastCommand) > timeout)
    {
        this->setBallHandling(0, 0);
        return true;
    }

    return false;
}

/**
 * Handles the ROS msg for changing the motor speeds.
 * @param msg BallHandleCmd
 */
void BallHandler::handleBallHandleControl(const msl_actuator_msgs::BallHandleCmd msg)
{
    this->updateTimeOfLastCmd();
    if (this->mode == msl_actuator_msgs::BallHandleMode::REMOTE_CONTROL)
    {
        this->setBallHandling(msg.leftMotor, msg.rightMotor);
    }
}

/**
 * Handles the ROS msg for changing the control mode of the motors.
 * @param msg BallHandleMode
 */
void BallHandler::handleBallHandleMode(const msl_actuator_msgs::BallHandleMode msg)
{
    this->updateTimeOfLastCmd();
    mode = msg.mode;
}
}
