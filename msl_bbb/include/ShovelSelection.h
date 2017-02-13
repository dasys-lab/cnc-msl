#pragma once

#include "Worker.h"

#include <BlackDef.h>
#include <BlackPWM.h>
#include <msl_actuator_msgs/ShovelSelectCmd.h>

#include <sys/time.h>

namespace BlackLib
{
class BlackPWM;
}

namespace msl_bbb
{

class ShovelSelection : public Worker
{
  public:
    /* API */
    // ShovelSelect(BeaglePWM::PwmPin pwm_name);

    ShovelSelection(BlackLib::pwmName pwm_P); // Delete if using API
    ~ShovelSelection();

    void run(); /** < overwrites the workers virtual run method */

    void updateTimeOfLastPositionChange();
    bool checkTimeout();
    void setShovel(bool passing);
    void handleShovelSelectControl(const msl_actuator_msgs::ShovelSelectCmd msg);

  private:
    /* API */
    // BeaglePWM *pwm;
    // BeaglePWM::PwmPin pwm_pin;

    BlackLib::BlackPWM *pwm; // Delete if using API

    const int period = 20000000;
    int kickPWM;
    int passPWM;

    bool enabled;
    bool statePassing;

    int timeout;
    timeval timeOfLastPositionChange;
};
}
