#pragma once

#include <Worker.h>


class BeagleGPIO;
class BeaglePins;

namespace supplementary
{
class SystemConfig;
}


namespace msl_bbb
{

enum SwitchPin
{
    sw_vision,
    sw_bundle,
    sw_power,
    led_power,
    led_bundle,
    led_vision
};

class Communication;
class Switches : public Worker
{
  public:
    Switches(Communication *comm);
    virtual ~Switches();

    void run();
    void readButtons();

  protected:
    supplementary::SystemConfig *sc;
    int ownID;
    BeagleGPIO *gpio;
    BeaglePins *pins;

    Communication *comm;
    uint8_t bundle_state;
    bool currentState[3];
    bool newState[3];
    int sw[3];
};

} /* namespace msl_bbb */
