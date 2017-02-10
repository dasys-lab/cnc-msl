#include "BBB.h"

#include "CanHandler.h"

#include <Configuration.h>
#include <SystemConfig.h>

#include <usbcanconnection.h>
#include <msl_actuator_msgs/BallHandleCmd.h>
#include <msl_actuator_msgs/BallHandleMode.h>
#include <msl_actuator_msgs/CanMsg.h>
#include <msl_actuator_msgs/IMUData.h>
#include <msl_actuator_msgs/MotionBurst.h>
#include <msl_actuator_msgs/MotionLight.h>
#include <msl_actuator_msgs/RawOdometryInfo.h>
#include <msl_actuator_msgs/ShovelSelectCmd.h>
#include <msl_actuator_msgs/VisionRelocTrigger.h>
#include <process_manager/ProcessCommand.h>
#include <std_msgs/Bool.h>

#include <exception>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string>
#include <unistd.h>
#include <chrono>

//mutex mtx;
//uint8_t th_count;
//bool th_activ = true;

void handleCanSub(const msl_actuator_msgs::CanMsg &msg)
{
    // Nachricht an ueber can verschicken
    canHandler.sendCanMsg(msg);
}

void run_udp()
{
    io_service.run();
}

void getLightbarrier()
{
    std_msgs::Bool msg;
    unique_lock<mutex> l_light(threw[2].mtx);
    while (th_activ)
    {
        threw[2].cv.wait(l_light, [&] { return !th_activ || threw[2].notify; }); // protection against spurious wake-ups
        if (!th_activ)
            return;

        try
        {
            msg.data = lightbarrier.checkLightBarrier();
            onRosBool2802967882(msg);
            // lbiPub->publish(msg);
        }
        catch (exception &e)
        {
            cout << "ADC: " << e.what() << endl;
        }

        threw[2].notify = false;
    }
}

void getSwitches()
{
    supplementary::SystemConfig *sc;
    sc = supplementary::SystemConfig::getInstance();
    enum Pin
    {
        sw_vision,
        sw_bundle,
        sw_power,
        led_power,
        led_bundle,
        led_vision
    };
    int ownID = (*sc)["bbb"]->get<int>("BBB.robotID", NULL);
    msl_actuator_msgs::VisionRelocTrigger msg_v;
    process_manager::ProcessCommand msg_pm;

    const char *pin_names[] = {"P9_11", "P9_13", "P9_15", "P9_23", "P9_41", "P9_42"}; /* sw_vis, sw_bun, sw_pwr, led_pwr, led_bun, led_vis */
    BeagleGPIO *gpio = BeagleGPIO::getInstance();
    BeaglePins *pins = gpio->claim((char **)pin_names, 6);

    int outputIdxs[] = {led_power, led_bundle, led_vision};
    pins->enableOutput(outputIdxs, 3);

    unique_lock<mutex> l_switches(threw[3].mtx);
    while (th_activ)
    {
        threw[3].cv.wait(l_switches, [&] { return !th_activ || threw[3].notify; }); // protection against spurious wake-ups
        if (!th_activ)
            return;

        static bool state[3] = {false, false, false};
        bool newstate[3];
        int sw[3] = {1, 1, 1};

        try
        {
            // TODO überprüfen, ob Auslesen mit der API funktioniert
            sw[sw_vision] = pins->getBit(sw_vision);
            sw[sw_bundle] = pins->getBit(sw_bundle);
            sw[sw_power] = pins->getBit(sw_power);
        }
        catch (exception &e)
        {
            cout << "Buttons: " << e.what() << endl;
        }

        for (int i = 0; i <= 2; i++)
        {
            if (sw[i] == 1)
                newstate[i] = false;
            else if (sw[i] == 0)
                newstate[i] = true;
            else
                cout << "Button " << i << " failure" << endl;
        }

        if (newstate[sw_bundle] != state[sw_bundle])
        {
            state[sw_bundle] = newstate[sw_bundle];

            if (state[sw_bundle])
            {
                static uint8_t bundle_state = 0;

                msg_pm.receiverId = ownID;
                msg_pm.robotIds = {ownID};
                msg_pm.processKeys = {2, 3, 4, 5, 7};
                msg_pm.paramSets = {1, 0, 0, 0, 3};

                if (bundle_state == 0)
                { // Prozesse starten
                    bundle_state = 1;
                    msg_pm.cmd = 0;
                    pins->setBit(led_bundle); // LED an
                }
                else if (bundle_state == 1)
                { // Prozesse stoppen
                    bundle_state = 0;
                    msg_pm.cmd = 1;
                    pins->clearBit(led_bundle); // LED aus
                }
                onRosProcessCommand554624761(msg_pm);
                // brtPub->publish(msg_pm);
            }
        }

        if (newstate[sw_vision] != state[sw_vision])
        {
            state[sw_vision] = newstate[sw_vision];

            if (state[sw_vision])
            {
                msg_v.receiverID = ownID;
                msg_v.usePose = false;
                onRosVisionRelocTrigger2772566283(msg_v);
                // vrtPub->publish(msg_v);
                pins->setBit(led_vision); // Vision-LED an
            }
            else
            {
                pins->clearBit(led_vision); // Vision-LED aus
            }
        }

        if (newstate[sw_power] != state[sw_power])
        {
            state[sw_power] = newstate[sw_power];

            if (state[sw_power])
            {
                std_msgs::Empty msg;
                // TODO not sent yet -> copy from generated code!
                // flPub->publish(msg);
                pins->setBit(led_power); // Power-LED an
            }
            else
            {
                pins->clearBit(led_power); // Power-LED aus
            }
        }

        threw[3].notify = false;
    }
    delete gpio;
}

void getIMU()
{
    unique_lock<mutex> l_imu(threw[4].mtx);
    while (th_activ)
    {
        threw[4].cv.wait(l_imu, [&] { return !th_activ || threw[4].notify; }); // protection against spurious wake-ups
        if (!th_activ)
            return;

        msl_actuator_msgs::IMUData msg;
        try
        {
            lsm9ds0.getData(time_now);
            msg = lsm9ds0.sendData(time_now);
            onRosIMUData3455796956(msg);
        }
        catch (exception &e)
        {
            cout << "IMU: " << e.what() << endl;
        }

        threw[4].notify = false;
    }
}

namespace msl_bbb
{
bool BBB::running = false;

BBB::BBB()
{
    ros::Time::init();

    // I2C
    bool i2c = myI2C.open(BlackLib::ReadWrite);
    bool spi = mySpi.open(BlackLib::ReadWrite);

    ;


    // Configure BallHandler Worker
    this->ballHandler = new BallHandler();
    this->ballHandler->msDelayedStart = std::chrono::milliseconds(0);
    this->ballHandler->msInterval = std::chrono::milliseconds(30);
    this->ballHandler->start();

    this->shovelSelection = new ShovelSelection();
    this->shovelSelection->start();

    OF_pins = {"P9_30", "P9_25", "P9_27", "P9_12"};
    this->opticalFlow = new OpticalFlow(OF_pins, &mySpi, comm);
    this->opticalFlow->adns_init();
    this->opticalFlow->start();

    thread th_lightbarrier(getLightbarrier);
    thread th_switches(getSwitches);

    IMU_pins = {"P8_11", "P8_15", "P8_17", "P8_26"}
    thread th_imu(getIMU);

    bool imu = lsm9ds0.init();
}

/**
 * The main working method, executed by the main thread.
 */
void BBB::run()
{
    ros::Rate loop_rate(30); // in Hz
    while (!running)
    {
        gettimeofday(&time_now, NULL);

        // Thread Notify
        for (int i = 0; i < 6; i++) // TODO remove magic number '6'
        {
            if (threw[i].notify)
            {
                cerr << "Thread " << i << " requires to much time, iteration is skipped" << endl;
            }
            else
            {
                threw[i].notify = true;
            }
            threw[i].cv.notify_all();
        }

        loop_rate.sleep();
    }
    io_service.stop();
    iothread.join();
}

/**
 * Handles ctrl+c commands from the kernel.
 * @param sig SIGINT
 */
void BBB::sigIntHandler(int sig)
{
    running = false;
    th_activ = false;

    for (int i = 0; i < 7; i++)
        threw[i].cv.notify_all();
}
}

int main(int argc, char **argv)
{
    // create BBB object
    msl_bbb::BBB beagleBoardControl;

    // register ctrl+c handler
    signal(SIGINT, msl_bbb::BBB::sigIntHandler);

    // start running
    beagleBoardControl.run();

    return 0;
}
