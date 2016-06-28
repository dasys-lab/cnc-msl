/*
 * Monitoring.cpp
 *
 *  Created on: 24 Mar 2016
 *      Author: emmeda
 */

#include "Monitoring.h"
#include <thread>
#include "MSLWorldModel.h"
#include <engine/AlicaEngine.h>
#include "RawSensorData.h"
#include <chrono>

namespace msl
{

	Monitoring::Monitoring(MSLWorldModel* wm) :
			wm(wm), running(true), isUsingSimulator(false)
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

} /* namespace msl */
