#include <BeaglePWM.h>
#include <ShovelSelection.h>
#include <SystemConfig.h>

namespace msl_bbb
{
/* API */
//	ShovelSelect::ShovelSelect(BeaglePWM::PwmPin pwm_name) {
//		pwm = BeaglePWM::getInstance();
//
//		pwm_pin = pwm_name;
//		pwm->setPeriod(pwm_pin, period);
//		pwm->setRunState(pwm_pin, true);
//		pwm->setDutyCycle(pwm_pin, 0);
//
//		auto sc = supplementary::SystemConfig::getInstance();
//		this->kickPWM = (*sc)["bbb"]->get<int>("BBB.shovelKick", NULL);
//		this->passPWM = (*sc)["bbb"]->get<int>("BBB.shovelPass", NULL);
//		this->timeout = (*sc)["bbb"]->get<int>("BBB.timeout", NULL);
//
//		enabled = false;
//		init = false;
//	}

/* Delete Constructor if using API */
ShovelSelection::ShovelSelection(BlackLib::pwmName pwm_P) : Worker("ShovelSelection")
{
    this->pwm = new BlackLib::BlackPWM(pwm_P);
    this->pwm->setPeriodTime(period, BlackLib::nanosecond);
    this->pwm->setSpaceRatioTime(0, BlackLib::microsecond);
    this->pwm->setRunState(BlackLib::stop);

    auto sc = supplementary::SystemConfig::getInstance();
    this->timeout = (*sc)["bbb"]->get<int>("BBB.timeout", NULL);
    this->kickPWM = (*sc)["bbb"]->get<int>("BBB.shovelKick", NULL);
    this->passPWM = (*sc)["bbb"]->get<int>("BBB.shovelPass", NULL);

    this->enabled = false;

    // default is short shovel for passing
    this->statePassing = true;
}

ShovelSelection::~ShovelSelection()
{
    /* API */
    // pwm->setRunState(pwm_pin, false);
	this->pwm->setRunState(BlackLib::stop);
    delete pwm;
}

/**
 * Checks when the last time updateTimeOfLastCmd() was called and
 * sets the PWMs runState to stop, if the timeout happened.
 */
bool ShovelSelection::checkTimeout()
{
    timeval t;
    gettimeofday(&t, NULL);
    if ((TIMEDIFFMS(t, timeOfLastPositionChange) > timeout) && enabled)
    {
        /* API */
        // pwm->setDutyCycle(pwm_pin, 0);
    	this->pwm->setRunState(BlackLib::stop); // Delete if using API
    	this->enabled = false;

        return true;
    }

    return false;
}

/**
 * Sets the shovel state to passing or lob shot.
 * Does not set anything if the
 * PWM-RunState is "run" AND the state is already set.
 * @param passing
 */
void ShovelSelection::setShovel(bool passing)
{
    if (statePassing == passing && enabled)
    {
    	return;
    }

	this->updateTimeOfLastPositionChange();

    if (passing)
    {
        /* API */
        /* 1000 because ns needed and passPWM is in us */
        // pwm->setDutyCycle(pwm_pin, passPWM * 1000);
        pwm->setSpaceRatioTime(passPWM, BlackLib::microsecond); // Delete if using API
    }
    else
    {
        /* API */
        /* 1000 because ns needed and kickPWM is in us */
        // pwm->setDutyCycle(pwm_pin, kickPWM * 1000);
        pwm->setSpaceRatioTime(kickPWM, BlackLib::microsecond); // Delete if using API
    }

    if (!enabled)
    {
        pwm->setRunState(BlackLib::run); // Delete if using API
        enabled = true;
    }

    // remember state set
    statePassing = passing;
}

/**
 * Handles ROS msgs for selecting the shovel state.
 * @param msg ShovelSelectCmd
 */
void ShovelSelection::handleShovelSelectControl(const msl_actuator_msgs::ShovelSelectCmd msg)
{
    try
    {
        this->setShovel(msg.passing);
    }
    catch (exception &e)
    {
        cout << "ShovelSelection: " << e.what() << endl;
    }
}

/**
 * Sets current time to timeOfLastCommand. This method is called
 * by every ROS msg receiving method.
 */
void ShovelSelection::updateTimeOfLastPositionChange()
{
    gettimeofday(&timeOfLastPositionChange, NULL);
}

void ShovelSelection::run()
{
    this->checkTimeout();
}
}
