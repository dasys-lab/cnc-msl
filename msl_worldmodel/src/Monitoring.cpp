/*
 * Monitoring.cpp
 *
 *  Created on: 24 Mar 2016
 *      Author: emmeda
 */

#include "Monitoring.h"
#include "MSLWorldModel.h"
#include "RawSensorData.h"
#include <chrono>
#include <engine/AlicaEngine.h>
#include <thread>

namespace msl
{

Monitoring::Monitoring(MSLWorldModel *wm)
    : wm(wm)
    , running(true)
    , isUsingSimulator(false)
    , oldMotionPosX(0)
    , oldMotionPosY(0)
    , oldVisionPosX(0)
    , oldVisionPosY(0)
    , errorCounter(0)
    , oldTime(wm->getTime())
{
    this->isUsingSimulator = wm->isUsingSimulator();
    // TODO make Monitoring.conf configuration
    this->monitorThread = new std::thread(&Monitoring::run, this);
}

Monitoring::~Monitoring()
{
    this->monitorThread->join();
    delete this->monitorThread;
}

void Monitoring::stop()
{
    this->running = false;
}

void Monitoring::run()
{
    while (this->running && ros::ok())
    {
        this->monitorSimulator();
        this->monitorMotion();
        this->looseWheel();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

void Monitoring::monitorSimulator()
{
    bool wmUsingSim = this->wm->isUsingSimulator();
    if (this->isUsingSimulator != wmUsingSim)
    {
        std::cout << "Mon: Simulator is " << (wmUsingSim ? "on." : "off.") << std::endl;
        this->isUsingSimulator = wmUsingSim;
    }
}

void Monitoring::monitorMotion()
{
    if (this->isUsingSimulator || wm->rawSensorData->getOwnPositionMotion())
    {
        this->setMaySendMessages(true);
    }
    else
    {
        this->setMaySendMessages(false);
    }
}

void Monitoring::setMaySendMessages(bool maySend)
{
    if (wm->getEngine() != nullptr && wm->getEngine()->isMaySendMessages() != maySend)
    {
        std::cout << "Mon: " << (maySend ? "Start" : "Stop") << " ALICA Engine to send messages." << std::endl;
        wm->getEngine()->setMaySendMessages(maySend);
    }
    if (wm->isMaySendMessages() != maySend)
    {
        std::cout << "Mon: " << (maySend ? "Start" : "Stop") << " WM to send messages." << std::endl;
        wm->setMaySendMessages(maySend);
    }
}

//Detection of a loose Wheel; exceeding a threshold three times in a row causes an error message
void Monitoring::looseWheel()
{
    if (!this->isUsingSimulator)
    {
        if (this->wm->rawSensorData->getOwnPositionMotion() == nullptr || this->wm->rawSensorData->getOwnPositionVision() == nullptr)
        {
            return;
        }

        if (wm->getTime() - oldTime >= 1000000000)
        {
            if (this->looseWheelCalc() > 750)
            {
                // std::cout << "loose wheel error: " << this->looseWheelCalc() << std::endl;
                errorCounter++;

                if (errorCounter >= 3)
                {
                    std::cout << "error detected: check for loose wheel" << std::endl;
                    errorCounter = 0;
                }
            }
            else
            {
                errorCounter = 0;
            }
            // std::cout << "TESTerrorError: " << this->looseWheelCalc() << std::endl;
            oldMotionPosX = this->wm->rawSensorData->getOwnPositionMotion()->x;
            oldMotionPosY = this->wm->rawSensorData->getOwnPositionMotion()->y;
            oldVisionPosX = this->wm->rawSensorData->getOwnPositionVision()->x;
            oldVisionPosY = this->wm->rawSensorData->getOwnPositionVision()->y;
            oldTime = wm->getTime();
        }
    }
}


//euclidean distance between vision and motion pos in a specified time interval
double Monitoring::looseWheelCalc()
{
    return sqrt(
        (this->wm->rawSensorData->getOwnPositionMotion()->x - this->wm->rawSensorData->getOwnPositionVision()->x - (oldMotionPosX - oldVisionPosX)) *
            (this->wm->rawSensorData->getOwnPositionMotion()->x - this->wm->rawSensorData->getOwnPositionVision()->x - (oldMotionPosX - oldVisionPosX)) +
        (this->wm->rawSensorData->getOwnPositionMotion()->y - this->wm->rawSensorData->getOwnPositionVision()->y - (oldMotionPosY - oldVisionPosY)) *
            (this->wm->rawSensorData->getOwnPositionMotion()->y - this->wm->rawSensorData->getOwnPositionVision()->y - (oldMotionPosY - oldVisionPosY)));
}

} /* namespace msl */
