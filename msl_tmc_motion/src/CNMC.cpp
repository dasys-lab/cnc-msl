/*
 * CNMCTriForce.cpp
 *
 *  Created on: Aug 26, 2015
 *      Author: cnpaul
 */

#include <CNMC.h>

namespace msl_driver
{

	CNMC::CNMC()
	{
		this->mc = nullptr;
		p = nullptr;
	}

	CNMC::~CNMC()
	{
		delete mc;
		delete p;
	}

	bool CNMC::initialize()
	{
		this->sc = supplementary::SystemConfig::getInstance();

		this->driverName = "CNMC";
		this->driverVersion = "0.0.1";

		this->minCycleTime = (*sc)["Motion"]->tryGet<long>(1, "Motion", "CNMC", "MinCycleTime", NULL) * 1000;

		this->device = (*sc)["Motion"]->get<string>("Motion", "CNMC", "Device", NULL);
		this->listen = (*sc)["Motion"]->tryGet<string>("udp://127.0.0.1/20100", "Motion", "CNMC", "Listen", NULL);

		this->initReadTimeout = (*sc)["Motion"]->tryGet<int>(1500, "Motion", "CNMC", "InitReadTimeout", NULL);
		this->readTimeout = (*sc)["Motion"]->tryGet<int>(250, "Motion", "CNMC", "ReadTimeout", NULL);
		this->writeTimeout = (*sc)["Motion"]->tryGet<int>(250, "Motion", "CNMC", "WriteTimeout", NULL);

		this->radius = (*sc)["Motion"]->get<double>("Motion", "CNMC", "RobotRadius", NULL);

		if (this->radius < 10)
		{
			std::cerr << "ROBOT RADIUS TOO LOW!!!" << std::endl;
		}

		this->maxVelocity = (*sc)["Motion"]->get<double>("Motion", "CNMC", "MaxVelocity", NULL);

		if (this->maxVelocity < 1)
		{
			std::cerr << "MAX VELOCITY TOO LOW!!" << std::endl;
			//TODO throw exception?

		}

		this->logOdometry = (*sc)["Motion"]->get<bool>("Motion", "CNMC", "LogOdometry", NULL);

//		WAS DAS??? TODO
//		this.logTypes = this.sc["Motion"].TryGetAllStrings(this.logTypes,"Motion","CNMC","LogType");

		getMotorConfig();

		this->runThread = std::thread(run, this);
	}

	void CNMC::open()
	{
		int stopBits = 1;
		int parity = 00;
		memset(&newtio, 0, sizeof newtio);
		if (this->device.find("/dev") == 0)
		{
			this->useSerial = true;
			this->port = open(this->device.c_str(), O_RDWR | O_NOCTTY);

			/* Error Handling */
			if (tcgetattr(port, &newtio) != 0)
			{
				std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
			}

			cfsetispeed(&newtio, (speed_t)B57600);
			cfsetospeed(&newtio, (speed_t)B57600);

			/* Setting other Port Stuff */
			newtio.c_cflag &= ~PARENB; // Make 8n1
			newtio.c_cflag &= ~CSTOPB;
			newtio.c_cflag &= ~CSIZE;
			newtio.c_cflag |= CS8;

			newtio.c_cflag &= ~CRTSCTS; // no flow control
			newtio.c_cc[VMIN] = 1; // read doesn't block
			newtio.c_cc[VTIME] = this->initReadTimeout; // 0.5 seconds read timeout
			newtio.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines

			//TODO Write timeout
			/* Make raw */
			cfmakeraw(&newtio);

			tcflush(port, TCIFLUSH);
			if (tcsetattr(port, TCSANOW, &newtio) != 0)
			{
				std::cout << "Error " << errno << " from tcsetattr" << std::endl;
			}
		}

		p = new CNMCPacketRequest();
		p->setData(CNMCPacket::RequestCmd::BatteryVoltage);

		sendData(p);

	}
	void CNMC::shutdownInternal()
	{
		if (this->useSerial)
		{
			if (tcsetattr(port, TCSANOW, &newtio) != 0)
			{
				close(this->port);
				this->port = 0;
			}
		}
	}
	void CNMC::executeCheck()
	{
		p = new CNMCPacketRequest();
		p->setData(CNMCPacket::RequestCmd::PathVector);

		sendData(p);
//		UpdateMotorState(null,(CNMCPacketRequestResponse)ReadData());

		if (this->stepCounter++ >= 80)
		{
			p = new CNMCPacketRequest();
			this->stepCounter = 0;
			p->setData(CNMCPacket::RequestCmd::BatteryVoltage);
			sendData(p);
			CNMCPacket r = readData();
//			UpdateMotorState(null,(CNMCPacketRequestResponse)r);
		}

	}
	void CNMC::updateMotorState(DriverData request, CNMCPacketRequestResponse* vmcp)
	{
		int rawMotorValues[3];

		switch ((CNMCPacketRequest::RequestCmd)vmcp->cmd)
		{
			case CNMCPacket::RequestCmd::PathVector:
			{
				if (vmcp->values->size() != 3)
				{
					std::cerr << "CNMCTriForce::updateMotorState --> CNMC velocity format error !" << std::endl;
				}

				//VAROBJECT in values!!! DEAL WITH IT
				if (vmcp->values->at(0).objectType == CNMCPacket::VAROBJECT::SHORT
						&& vmcp->values->at(1).objectType == CNMCPacket::VAROBJECT::SHORT
						&& vmcp->values->at(2).objectType == CNMCPacket::VAROBJECT::SHORT)
				{
					rawMotorValues[0] = (short)vmcp->values->at(0).value.shValue;
					rawMotorValues[1] = (short)vmcp->values->at(1).value.shValue;
					;
					rawMotorValues[2] = (short)vmcp->values->at(2).value.shValue;
					;
				}
				else
				{
					std::cerr << "CNMCTriForce: vmcp->values->at is not short value" << std::endl;
				}

				MotionSet* mr = new MotionSet();
				mr->angle = atan2(rawMotorValues[1], rawMotorValues[0]);
				mr->translation = sqrt(rawMotorValues[0] * rawMotorValues[0] + rawMotorValues[1] * rawMotorValues[1]);
				mr->rotation = ((double)rawMotorValues[2]) / 64.0;

			}
				break;
			case CNMCPacket::RequestCmd::EncoderTicksRel:
			{

			}
				break;
			case CNMCPacket::RequestCmd::BatteryVoltage:
			{

			}
				break;
			default:
				break;
		}
	}
	void CNMC::signalResult(DriverData data)
	{
		//TODO
		//if(this->re)
	}
	void CNMC::addResult(DriverData data)
	{
		resultMutex.lock();
		for (int i = 0; i < this->results.size(); i++)
		{
			//TODO SHOULD BE TESTED
			if (typeid(this->results[i]).name() == typeid(data).name())
			{
				data.timespamp = (clock() / 10000) + 3000;
				this->results[i] = data;
				signalResult(data);
				return;
			}
		}
		this->results.push_back(data);
		signalResult(data);
		resultMutex.unlock();
	}
	void CNMC::getMotorConfig()
	{
		this->mc = new MotorConfig();
		this->mc->resolution = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Motors", "EncoderResolution", NULL);
		this->mc->maxSpeed = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Motors", "MaxMotorSpeed", NULL);
		this->mc->maxCurrent = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Motors", "MaxCurrent", NULL);
		this->mc->limitedCurrent = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Motors", "LimitedCurrent", NULL);
		this->mc->wheelRadius = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Motors", "WheelRadius", NULL);
		this->mc->gearReduction = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Motors", "GearReduction", NULL);

		//Controller values
		this->mc->pidKp = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller", "PIDKp", NULL);
		this->mc->pidB = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller", "PIDB", NULL);
		this->mc->pidKi = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller", "PIDKi", NULL);
		this->mc->pidKd = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller", "PIDKd", NULL);
		this->mc->pidKdi = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller", "PIDKdi", NULL);
		this->mc->linFactor = (*sc)["Motion"]->tryGet<double>(0.0, "Motion", "CNMC", "Controller", "LinearFactor",
																NULL);
		this->mc->smoothFactor = (*sc)["Motion"]->tryGet<double>(1.0, "Motion", "CNMC", "Controller", "SmoothFactor",
																	NULL);

		this->mc->maxErrorInt = (*sc)["Motion"]->tryGet<short>(1000, "Motion", "CNMC", "Controller", "MaxErrorInt",
																NULL);

		this->mc->rotationErrorWeight = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller",
																		"RotationErrorWeight", NULL);
		this->mc->rotationErrorByVeloWeight = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller",
																			"RotationErrorByVeloWeight", NULL);
		this->mc->rotationErrorByAccelWeight = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller",
																			"RotationErrorByAccelWeight", NULL);

		this->mc->deadBand = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Controller", "DeadBand", NULL);

		this->mc->failSafeRPMBound = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Controller", "FailSafeRPMBound",
																	NULL);
		this->mc->failSafePWMBound = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Controller", "FailSafePWMBound",
																	NULL);
		this->mc->failSafeCycles = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Controller", "FailSafeCycles", NULL);

		this->mc->maxAcceleration = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller", "MaxAcceleration",
																	NULL);
		this->mc->maxDecceleration = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller", "MaxDecceleration",
																	NULL);
		this->mc->maxRotForce = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller", "MaxRotForce", NULL);

		this->mc->rotationAccelBound = (*sc)["Motion"]->tryGet<double>(0.0, "Motion", "CNMC", "Controller",
																		"MaxRotationAccel");

		this->mc->currentErrorBound = (*sc)["Motion"]->tryGet<short>(0, "Motion", "CNMC", "Controller",
																		"CurrentErrorBound", NULL);
		if (this->mc->currentErrorBound == 0)
		{
			this->mc->controlCurrent = false;
		}
		else
		{
			this->mc->controlCurrent = true;
			this->mc->currentKp = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller", "CurrentKp", NULL);
			this->mc->currentKi = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller", "CurrentKi", NULL);
			this->mc->currentKd = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller", "CurrentKd", NULL);
		}

		this->mc->denominator = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Motors",
															"ThermalMotorConstantDenominator", NULL);
		this->mc->numerator = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Motors", "ThermalMotorConstantNumerator",
															NULL);

		int delta = (int)(((2.0 * M_PI * (double)mc->wheelRadius)
				/ (((double)mc->resolution) * (double)mc->gearReduction)) * 1000.0);

		this->mc->vmax = (delta * mc->resolution * 2.0 * mc->maxSpeed) / (60.0 * 1000.0);
	}

	void CNMC::sendData(CNMCPacket* packet)
	{
		auto bytes = packet->getBytes();
		if (this->useSerial)
		{
			try
			{
			}
			catch (std::exception e)
			{
				shutdownInternal();
				cout << "Could not write on port " << e.what() << endl;
			}
		}
		else
		{
			try
			{
				this->writeTimeoutCount = 0;
			}
			catch (std::exception e)
			{
				shutdownInternal();
				cout << "network write timeout " << this->writeTimeoutCount++ << " " << e.what() << endl;
			}
		}
	}

	CNMCPacket CNMC::readData()
	{
	}
	void CNMC::run()
	{

		// Loop until the driver is closed
		while (!this->quit)
		{

			// TODO strange thing...
			//PurgeResults();

			// If the driver is not active try to open it
			if (!this->active)
			{
				this->open();
			}

			if (this->active)
			{

				// 1 Tick = 100ns, 10 Ticks = 1us
				// remember the time, processing was last triggered

				this.deltaTime = DateTime.UtcNow.Ticks - this.cycleLastTimestamp;
				this.cycleLastTimestamp += deltaTime;

				DriverData request = null;

				// Get the next request from the queue
				lock(this.dataBufferLock)
				{

					request = this.dataBuffer;
					this.dataBuffer = null;

				}

				if (request != null)
				{
					// If there is a request, try to process it

					try
					{
						// Set the internal status to StateCode.ExecutingRequest
						SignalStatus(StatusCode.ExecutingRequest, null);

						if (!this.quit)
						{
							ExecuteRequest(request);
						}

					}
					catch (Exception e)
					{

						Debug.WriteLine("Driver: error executing request: {0}", e.Message);
						SignalStatus(StatusCode.ErrorExecutingRequest, e.Message);

						Shutdown();

					}

				}
				else
				{
					// If there is no request, call the ExecuteCheck method

					try
					{
						// Set the internal status to StateCode.ExecutingCheck
						SignalStatus(StatusCode.ExecutingCheck, null);

						if (!this.quit)
						{
							ExecuteCheck();
						}

					}
					catch (Exception e)
					{

						Debug.WriteLine("Driver: error executing check: {0}", e.Message);
						SignalStatus(StatusCode.ErrorExecutingCheck, e.Message);

						Shutdown();

					}
				}

				// minCycleTime (us), Ticks (tick), cycleLastTimestamp (tick), 1 tick = 100 ns
				long sleepTime = (this.minCycleTime - (DateTime.UtcNow.Ticks - this.cycleLastTimestamp) / 10) / 1000;

				if (sleepTime > 0)
				{
					Thread.Sleep((int)sleepTime);
				}

				// Compute the new due time
				long newDueTime = this.alivePeriod - ((DateTime.UtcNow.Ticks - this.cycleLastTimestamp) / 10000);

				if (newDueTime <= 0)
				{
					newDueTime = 0;
				}

				// Schedule the timer
				//this.aliveTimer.Change(20, 0);
				this.aliveTimer.Change(newDueTime, 0);
			}

		}
	}

} /* namespace msl_driver */
