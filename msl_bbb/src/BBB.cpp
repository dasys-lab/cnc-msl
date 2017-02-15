#include "BBB.h"

#include "BallHandler.h"
#include "CanHandler.h"
#include "Communication.h"
#include "IMU.h"
#include "LightBarrier.h"
#include "OpticalFlow.h"
#include "ShovelSelection.h"
#include "Switches.h"

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

#include <BlackI2C.h>

#include <chrono>
#include <exception>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string>
#include <unistd.h>

namespace msl_bbb
{
bool BBB::running = false;

BBB::BBB()
{
    ros::Time::init();
    this->comm = new Communication();

    // Configure Ball Handler Worker
    this->ballHandler = new BallHandler();
    this->ballHandler->setDelayedStartMS(std::chrono::milliseconds(0));
    this->ballHandler->setIntervalMS(std::chrono::milliseconds(30));
    this->ballHandler->start();

    // Configure Shovel Selection Worker
    this->shovelSelection = new ShovelSelection(BlackLib::P9_14);
    this->shovelSelection->setDelayedStartMS(std::chrono::milliseconds(0));
    this->shovelSelection->setIntervalMS(std::chrono::milliseconds(30));
    this->shovelSelection->start();

    // Configure Optical Flow Sensor Worker
    this->opticalFlow = new OpticalFlow(comm);
    this->opticalFlow->adns_init();
    this->opticalFlow->setDelayedStartMS(std::chrono::milliseconds(0));
    this->opticalFlow->setIntervalMS(std::chrono::milliseconds(30));
    this->opticalFlow->start();

    // Configure Light Barrier Worker
    this->lightbarrier = new LightBarrier(BlackLib::AIN0, this->comm);
    this->lightbarrier->setDelayedStartMS(std::chrono::milliseconds(0));
    this->lightbarrier->setIntervalMS(std::chrono::milliseconds(30));
    this->lightbarrier->start();

    // Configure Switches Worker
    this->switches = new Switches(this->comm);
    this->switches->setDelayedStartMS(std::chrono::milliseconds(0));
    this->switches->setIntervalMS(std::chrono::milliseconds(30));
    this->switches->start();

    // Configure IMU Worker
    this->imu = new IMU(this->comm);
    this->imu->init();
    this->imu->setDelayedStartMS(std::chrono::milliseconds(0));
    this->imu->setIntervalMS(std::chrono::milliseconds(30));
    this->imu->start();
}

BBB::~BBB()
{
    delete this->imu;
    delete this->switches;
    delete this->lightbarrier;
    delete this->opticalFlow;
    delete this->shovelSelection;
    delete this->ballHandler;
}

/**
 * The main working method, executed by the main thread.
 */
void BBB::run()
{
    while (running)
    {
        sleep(1);
    }
}

/**
 * Handles ctrl+c commands from the kernel.
 * @param sig SIGINT
 */
void BBB::sigIntHandler(int sig)
{
    running = false;
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
