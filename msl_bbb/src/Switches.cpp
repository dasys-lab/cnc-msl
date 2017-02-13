#include "Switches.h"
#include "Communication.h"

#include <BeaglePins.h>
#include <BlackGPIO.h>
#include <SystemConfig.h>
#include <process_manager/ProcessCommand.h>
#include <msl_actuator_msgs/VisionRelocTrigger.h>

#include <vector>

namespace msl_bbb
{

Switches::Switches(Communication *comm)
    : Worker("Switches")
    , sw{1, 1, 1}
    , currentState{false, false, false}
    , newState{false, false, false}
{
    this->bundle_state = 0;
    this->comm = comm;
    this->sc = supplementary::SystemConfig::getInstance();
    ownID = (*sc)["bbb"]->get<int>("BBB.robotID", NULL);

    this->gpio = BeagleGPIO::getInstance();
    /* sw_vis, sw_bun, sw_pwr, led_pwr, led_bun, led_vis */
    std::vector<char const *> SW_pins = {"P9_11", "P9_13", "P9_15", "P9_23", "P9_41", "P9_42"};
    this->pins = this->gpio->claim((char **)SW_pins.data(), 6);

    int outputIdxs[] = {led_power, led_bundle, led_vision};
    pins->enableOutput(outputIdxs, 3);
}

Switches::~Switches()
{
    delete gpio;
}

void Switches::readButtons()
{
    // TODO überprüfen, ob Auslesen mit der API funktioniert
    this->sw[sw_vision] = this->pins->getBit(sw_vision);
    this->sw[sw_bundle] = this->pins->getBit(sw_bundle);
    this->sw[sw_power] = this->pins->getBit(sw_power);

    for (int i = 0; i <= 2; i++)
    {
        if (this->sw[i] == 1)
        {
            this->newState[i] = false;
        }
        else if (this->sw[i] == 0)
        {
            this->newState[i] = true;
        }
        else
        {
            std::cout << "Switches: Button " << i << " failure!" << std::endl;
        }
    }
}

void Switches::run()
{
	this->readButtons();

    // Processes Button
    if (newState[sw_bundle] != currentState[sw_bundle])
    {
        currentState[sw_bundle] = newState[sw_bundle];

        if (currentState[sw_bundle])
        {
            process_manager::ProcessCommand msg_pm;
            msg_pm.receiverId = this->ownID;
            msg_pm.robotIds = {this->ownID};
            msg_pm.processKeys = {2, 3, 4, 5, 7};
            msg_pm.paramSets = {1, 0, 0, 0, 3};

            if (bundle_state == 0)
            { // Start processes
                bundle_state = 1;
                msg_pm.cmd = 0;
                pins->setBit(led_bundle); // Process-LED on
            }
            else if (bundle_state == 1)
            { // Stop processes
                bundle_state = 0;
                msg_pm.cmd = 1;
                pins->clearBit(led_bundle); // Process-LED off
            }
            this->comm->onRosProcessCommand554624761(msg_pm);
        }
    }

    // Vision Relocalisation Button
    if (newState[sw_vision] != currentState[sw_vision])
    {
        currentState[sw_vision] = newState[sw_vision];

        if (currentState[sw_vision])
        {
            msl_actuator_msgs::VisionRelocTrigger msg_v;
            msg_v.receiverID = ownID;
            msg_v.usePose = false;
            this->comm->onRosVisionRelocTrigger2772566283(msg_v);
            pins->setBit(led_vision); // Vision-LED on
        }
        else
        {
            pins->clearBit(led_vision); // Vision-LED off
        }
    }

    // PowerSwitch Beeper Button
    if (newState[sw_power] != currentState[sw_power])
    {
        currentState[sw_power] = newState[sw_power];

        if (newState[sw_power])
        {
            // TODO not sent yet -> copy from generated code!
            // std_msgs::Empty msg;
            // this->comm->onRosWhatever(msg);
            pins->setBit(led_power); // Power-LED on
        }
        else
        {
            pins->clearBit(led_power); // Power-LED off
        }
    }
}
} /* namespace msl_bbb */
