/*
 * Motion.cpp
 *
 *  Created on: Aug 19, 2015
 *      Author: Stephan Opfer
 */

#include "Motion.h"
#include <thread>
#include <signal.h>
#include <SystemConfig.h>
#include <time.h>

namespace msl_driver
{

	bool Motion::running;

	Motion::Motion(int argc, char** argv) :
			rosNode(nullptr), spinner(nullptr)
	{

		this->motionValue = new MotionSet();
		this->motionResult = new MotionSet();

		this->sc = supplementary::SystemConfig::getInstance();

		this->ownId = this->sc->getOwnRobotID();

		this->odometryDelay = (*sc)["Motion"]->tryGet<int>(100000, "Motion", "OdometryDelay", NULL);
		// Read slip control parameters
		this->slipControlEnabled = (*sc)["Motion"]->tryGet<bool>(false, "Motion", "SlipControl", "Enabled", NULL);
		this->slipControlMinSpeed = (*sc)["Motion"]->tryGet<double>(1250.0, "Motion", "SlipControl", "MinSpeed", NULL);
		this->slipControlDiffAngle = (*sc)["Motion"]->tryGet<double>((M_PI / 180.0) * 10.0, "Motion", "SlipControl",
																		"DiffAngle", NULL);
		this->slipControlDiffAngleMinSpeed = (*sc)["Motion"]->tryGet<double>(400.0, "Motion", "SlipControl",
																				"DiffAngleMinSpeed", NULL);
		this->slipControlOldMaxRot = (*sc)["Motion"]->tryGet<double>(M_PI / 20.0, "Motion", "SlipControl", "OldMaxRot",
		NULL);
		this->slipControlNewMinRot = (*sc)["Motion"]->tryGet<double>(M_PI / 2.0, "Motion", "SlipControl", "NewMinRot",
		NULL);

		if (this->slipControlEnabled)
		{
			std::cout << "Motion: slip control min speed            = " << this->slipControlMinSpeed << std::endl;
			std::cout << "Motion: slip control diff angle           = " << this->slipControlDiffAngle << std::endl;
			std::cout << "Motion: slip control diff angle min speed = " << this->slipControlDiffAngleMinSpeed
					<< std::endl;
			std::cout << "Motion: slip control old max rotation     = " << this->slipControlOldMaxRot << std::endl;
			std::cout << "Motion: slip control new min rotation     = " << this->slipControlNewMinRot << std::endl;
		}

		// Read acceleration compensation parameters
		this->accelCompEnabled = (*sc)["Motion"]->tryGet<bool>(false, "Motion", "AccelComp", "Enabled", NULL);
		this->accelCompMaxAccel = (*sc)["Motion"]->tryGet<double>(4000.0, "Motion", "AccelComp", "MaxAccel", NULL);
		this->accelCompMaxAngularAccel = (*sc)["Motion"]->tryGet<double>(M_PI / 2.0, "Motion", "AccelComp",
																			"MaxAngularAccel", NULL);

		if (this->accelCompEnabled)
		{
			std::cout << "Motion: max acceleration " << this->accelCompMaxAccel << std::endl;
			std::cout << "Motion: max angular acceleration " << this->accelCompMaxAngularAccel << std::endl;
		}

		// Initialize the acceleration compensation
		this->accelComp = new AccelCompensation(this->accelCompMaxAccel, this->accelCompMaxAngularAccel);

		// Set Trace-Model (according to the impera repository, we always used the CircleTrace model)
		this->traceModel = new CircleTrace();

		// Read required driver parameters
		this->driverAlivePeriod = (*sc)["Motion"]->tryGet<int>(250, "Motion", "AlivePeriod", NULL);
		this->driverOpenAttemptPeriod = (*sc)["Motion"]->tryGet<int>(1000, "Motion", "OpenAttemptPeriod", NULL);
	}

	Motion::~Motion()
	{
		delete motionValue;
		delete motionResult;
		delete motionValueOld;
		delete accelComp;
		delete traceModel;
	}

	/**
	 * Initialises all ROS communcation (sub and pub).
	 */
	void Motion::initCommunication(int argc, char** argv)
	{
		ros::init(argc, argv, "MSL_TMC_Motion");
		rosNode = new ros::NodeHandle();
		spinner = new ros::AsyncSpinner(4);
		handleMotionControlSub = rosNode->subscribe("/MotionControl", 10, &Motion::handleMotionControl, (Motion*)this);
		rawOdometryInfoPub = rosNode->advertise<msl_actuator_msgs::RawOdometryInfo>("/RawOdometry", 10);
		motionStatInfoPub = rosNode->advertise<msl_actuator_msgs::MotionStatInfo>("/MotionStatInfo", 10);
		spinner->start();
	}

	void Motion::initialize()
	{
		this->minCycleTime = (*sc)["Motion"]->tryGet<long>(1, "Motion", "CNMC", "MinCycleTime", NULL) * 1000;

		this->device = (*sc)["Motion"]->get<string>("Motion", "CNMC", "Device", NULL);

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
		}

		this->logOdometry = (*sc)["Motion"]->get<bool>("Motion", "CNMC", "LogOdometry", NULL);

		// copied that from Mops Motion.conf!
		this->logTypes = make_shared<vector<string>>(initializer_list<string> {"ERRORINT", "MGOAL", "MOTION"});
		this->logTypesAvailable = make_shared<vector<string>>(initializer_list<string> {"RPM", "PWM", "RPMGOAL",
																						"CURRENT", "MGOAL", "MREQUEST",
																						"MAXPWM", "ERRORINT", "MOTION",
																						"MSMOOTH", "RPMSMOOTH"});

		getMotorConfig();
	}

	void Motion::open()
	{
		//### SERIEAL PORT STUFF
		this->port = ::open(this->device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

		// Read Configuration into newtio
		memset(&newtio, 0, sizeof newtio); // newtio will contain the configuration of port
		if (tcgetattr(port, &newtio) != 0)
		{
			std::cerr << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
		}
		if (cfsetispeed(&newtio, (speed_t)B57600) < 0)
		{
			std::cerr << "Error " << errno << " from cfsetispeed: " << strerror(errno) << std::endl;
		}
		if (cfsetospeed(&newtio, (speed_t)B57600) < 0)
		{
			std::cerr << "Error " << errno << " from cfsetospeed: " << strerror(errno) << std::endl;
		}

		// Setting other Port Stuff
		newtio.c_cflag &= ~(CSIZE | PARENB | CSTOPB); // Make 8n1
		newtio.c_cflag |= CS8;

		newtio.c_cflag &= ~CRTSCTS; // no flow control
		newtio.c_cc[VMIN] = 1; // read doesn't block
		newtio.c_cc[VTIME] = this->initReadTimeout; // 0.5 seconds read timeout
		newtio.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines

		/* Make raw */
		cfmakeraw(&newtio);

		tcflush(port, TCIFLUSH);
		if (tcsetattr(port, TCSANOW, &newtio) < 0)
		{
			std::cerr << "Error " << errno << " from tcsetattr" << std::endl;
		}

		//### SEND MOTOR CONFIG
		this->sendMotorConfig();

		//### READY
		this->controllerIsActive = true;
	}

	void Motion::sendData(shared_ptr<CNMCPacket> packet)
	{
		auto bytes = packet->getBytes();
		::write(this->port, (*bytes).data(), bytes->size());
	}

	CNMCPacket* Motion::readData()
	{
		uint8_t b;
		bool finished = false;
		bool quoted = false;

		vector<uint8_t> data;
		::read(this->port, &b, 1);
		bool wrote = false;
		while (b != CNMCPacket::START_HEADER)
		{
			cout << (char)b;
			::read(this->port, &b, 1);
			wrote = true;
		}
		if (wrote)
			cout << endl;

		if (b != CNMCPacket::START_HEADER)
		{
			return nullptr;
		}
		else
		{
			data.push_back(b);
		}

		while (!finished)
		{
			::read(this->port, &b, 1);

			if (b == CNMCPacket::QUOTE)
			{
				if (!quoted)
					quoted = true;
				else
				{
					data.push_back(b);
					quoted = false;
				}
			}
			else if (b == CNMCPacket::END_HEADER)
			{
				if (!quoted)
				{ //do not add end header to data
					finished = true;
				}
				else
				{
					data.push_back(b);
					quoted = false;
				}
			}
			else
			{
				quoted = false;
				data.push_back(b);
			}
		}

		return CNMCPacket::getInstance(data.data(), data.size());
	}

	void Motion::checkSuccess(shared_ptr<CNMCPacket> cmd)
	{
		CNMCPacket result = readData();
		if (!cmd->isExpectedResponse(result))
		{
			// TODO implement toString of CNMCPacket
			//cerr << "Error setting CNMC " << cmd.toString() << "\n Response was " << result.toString() << endl;
		}
	}

	void Motion::sendMotorConfig()
	{
		shared_ptr<CNMCPacketConfigure> configPacket;

		//gear ratio
		configPacket = make_shared<CNMCPacketConfigure>();
		shared_ptr<vector<uint8_t>> values = make_shared<vector<uint8_t>>();
		values->push_back(1);
		values->push_back((uint8_t)this->mc.gearReduction);
		configPacket->setData(CNMCPacket::ConfigureCmd::GearRatio, values);
		this->sendData(configPacket);
		this->checkSuccess(configPacket);

		//encoder ticks per (half) rotation
		configPacket = make_shared<CNMCPacketConfigure>();
		configPacket->setData(CNMCPacket::ConfigureCmd::EncoderTicksPerRot, (short)this->mc.resolution);
		this->sendData(configPacket);
		this->checkSuccess(configPacket);

		//wheel Radius
		configPacket = make_shared<CNMCPacketConfigure>();
		configPacket->setData(CNMCPacket::ConfigureCmd::WheelRadius, (short)(this->mc.wheelRadius * 10));
		this->sendData(configPacket);
		this->checkSuccess(configPacket);

		//Robot Radius
		configPacket = make_shared<CNMCPacketConfigure>();
		configPacket->setData(CNMCPacket::ConfigureCmd::RobotRadius, (short)this->radius);
		this->sendData(configPacket);
		this->checkSuccess(configPacket);

		//maxRPM
		int result = (int)(this->mc.maxSpeed / this->mc.gearReduction);
		configPacket = make_shared<CNMCPacketConfigure>();
		configPacket->setData(CNMCPacket::ConfigureCmd::MaxRPM, (short)result);
		this->sendData(configPacket);
		this->checkSuccess(configPacket);

		shared_ptr<CNMCPacketCtrlConfigure> ctrlPacket;
		short tmp;

		//PIDKp
		ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
		tmp = (short)std::round(8192 * fmin(3, fmax(-3, this->mc.pidKd)));
		ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::PIDKp, tmp);
		this->sendData(ctrlPacket);
		this->checkSuccess(ctrlPacket);

		//PIDKb
		ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
		tmp = (short)std::round(8192 * fmin(3, fmax(-3, this->mc.pidB)));
		ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::PIDb, tmp);
		this->sendData(ctrlPacket);
		this->checkSuccess(ctrlPacket);

		//PIDKi
		ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
		tmp = (short)std::round(8192 * fmin(3, fmax(-3, this->mc.pidKi)));
		ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::PIDKi, tmp);
		this->sendData(ctrlPacket);
		this->checkSuccess(ctrlPacket);

		//PIDKd
		ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
		tmp = (short)std::round(8192 * fmin(3, fmax(-3, this->mc.pidKd)));
		ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::PIDKd, tmp);
		this->sendData(ctrlPacket);
		this->checkSuccess(ctrlPacket);

		//PIDKdi
		ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
		tmp = (short)std::round(8192 * fmin(3, fmax(-3, this->mc.pidKdi)));
		ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::PIDKdi, tmp);
		this->sendData(ctrlPacket);
		this->checkSuccess(ctrlPacket);

		//linFactor
		ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
		tmp = (short)std::round(8192 * fmin(3, fmax(-3, this->mc.linFactor)));
		ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::LinearFactor, tmp);
		this->sendData(ctrlPacket);
		this->checkSuccess(ctrlPacket);

		//smoothFactor
		ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
		tmp = (short)std::round(8192 * fmin(3, fmax(-3, this->mc.smoothFactor)));
		ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::SmoothFactor, tmp);
		this->sendData(ctrlPacket);
		this->checkSuccess(ctrlPacket);

		//maxErrorInt
		ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
		ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::MaxErrorInt, (short)this->mc.maxErrorInt);
		this->sendData(ctrlPacket);
		this->checkSuccess(ctrlPacket);

		//Rotation Error Weight
		ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
		tmp = (short)std::round(8192 * fmin(3, fmax(-3, this->mc.rotationErrorWeight)));
		ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::RotationErrorW, tmp);
		this->sendData(ctrlPacket);
		this->checkSuccess(ctrlPacket);

		ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
		tmp = (short)std::round(8192 * fmin(3, fmax(-3, this->mc.rotationErrorByVeloWeight)));
		ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::RotationErrorVeloW, tmp);
		this->sendData(ctrlPacket);
		this->checkSuccess(ctrlPacket);

		ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
		tmp = (short)std::round(8192 * fmin(3, fmax(-3, this->mc.rotationErrorByAccelWeight)));
		ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::RotationErrorAccelW, tmp);
		this->sendData(ctrlPacket);
		this->checkSuccess(ctrlPacket);

		//DeadBand
		ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
		ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::DeadBand, (short)this->mc.deadBand);
		this->sendData(ctrlPacket);
		this->checkSuccess(ctrlPacket);

		//Lower Accel Bound
		ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
		ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::LowerAccelBound, (short)this->mc.accelBoundMin);
		this->sendData(ctrlPacket);
		this->checkSuccess(ctrlPacket);

		//Higher Accel Bound
		ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
		ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::HigherAccelBound, (short)this->mc.accelBoundMax);
		this->sendData(ctrlPacket);
		this->checkSuccess(ctrlPacket);

		//Max Rotation Acceleration
		ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
		tmp = (short)std::round(64 * this->mc.rotationAccelBound);
		ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::MaxRotationAccel, tmp);
		this->sendData(ctrlPacket);
		this->checkSuccess(ctrlPacket);

		//Fail Safe
		ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
		shared_ptr<vector<short>> vals = make_shared<vector<short>>();
		vals->push_back((short)this->mc.failSafeRPMBound);
		vals->push_back((short)this->mc.failSafePWMBound);
		vals->push_back((short)this->mc.failSafeCycles);
		ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::FailSafeValues, vals);
		this->sendData(ctrlPacket);
		this->checkSuccess(ctrlPacket);

		//Current Control:
		if (this->mc.controlCurrent)
		{
			ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
			ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::CurrentErrorBound, (short)this->mc.currentErrorBound);
			this->sendData(ctrlPacket);
			this->checkSuccess(ctrlPacket);

			ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
			ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::CurrentKp, (short)this->mc.currentKp);
			this->sendData(ctrlPacket);
			this->checkSuccess(ctrlPacket);

			ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
			ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::CurrentKi, (short)this->mc.currentKi);
			this->sendData(ctrlPacket);
			this->checkSuccess(ctrlPacket);

			ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
			ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::CurrentKd, (short)this->mc.currentKd);
			this->sendData(ctrlPacket);
			this->checkSuccess(ctrlPacket);
		}

		// TODO
		int8_t logmode = 0x00;
		for (int i = 0; i < this->logTypesAvailable->size(); i++)
		{
			auto iter = std::find(this->logTypes->begin(), this->logTypes->end(), this->logTypesAvailable->at(i));
			if (iter == this->logTypes->end())
			{
				logmode = -1;
			}
			else
			{
				logmode = (int8_t)std::distance(this->logTypes->begin(), iter);
			}

			configPacket = make_shared<CNMCPacketConfigure>();
			configPacket->setData(CNMCPacket::ConfigureCmd::SetLogMode,
									make_shared<vector<int8_t>>((int8_t)i, logmode));
			this->sendData(configPacket);
			this->checkSuccess(configPacket);
		}

		//cycle time
		configPacket = make_shared<CNMCPacketConfigure>();
		configPacket->setData(CNMCPacket::ConfigureCmd::CycleTime, (short)5);
		this->sendData(configPacket);
		this->checkSuccess(configPacket);

		//COMMIT
		ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
		ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::Commit);
		this->sendData(ctrlPacket);
		this->checkSuccess(ctrlPacket);

		//cycle time
		configPacket = make_shared<CNMCPacketConfigure>();
		configPacket->setData(CNMCPacket::ConfigureCmd::Mode, (short)1);
		this->sendData(configPacket);
		this->checkSuccess(configPacket);

		//Toggle Logging
		configPacket = make_shared<CNMCPacketConfigure>();
		shared_ptr<vector<uint8_t>> valByte = make_shared<vector<uint8_t>>();
		valByte->push_back(this->logOdometry ? (uint8_t)1 : (uint8_t)0);
		configPacket->setData(CNMCPacket::ConfigureCmd::ToggleOdoLog, valByte);
		this->sendData(configPacket);
		this->checkSuccess(configPacket);
	}

	void Motion::getMotorConfig()
	{
		this->mc.resolution = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Motors", "EncoderResolution", NULL);
		this->mc.maxSpeed = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Motors", "MaxMotorSpeed", NULL);
		this->mc.maxCurrent = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Motors", "MaxCurrent", NULL);
		this->mc.limitedCurrent = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Motors", "LimitedCurrent", NULL);
		this->mc.wheelRadius = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Motors", "WheelRadius", NULL);
		this->mc.gearReduction = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Motors", "GearReduction", NULL);

		//Controller values
		this->mc.pidKp = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller", "PIDKp", NULL);
		this->mc.pidB = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller", "PIDB", NULL);
		this->mc.pidKi = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller", "PIDKi", NULL);
		this->mc.pidKd = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller", "PIDKd", NULL);
		this->mc.pidKdi = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller", "PIDKdi", NULL);
		this->mc.linFactor = (*sc)["Motion"]->tryGet<double>(0.0, "Motion", "CNMC", "Controller", "LinearFactor",
		NULL);
		this->mc.smoothFactor = (*sc)["Motion"]->tryGet<double>(1.0, "Motion", "CNMC", "Controller", "SmoothFactor",
		NULL);

		this->mc.maxErrorInt = (*sc)["Motion"]->tryGet<short>(1000, "Motion", "CNMC", "Controller", "MaxErrorInt",
		NULL);

		this->mc.rotationErrorWeight = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller",
																	"RotationErrorWeight", NULL);
		this->mc.rotationErrorByVeloWeight = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller",
																			"RotationErrorByVeloWeight", NULL);
		this->mc.rotationErrorByAccelWeight = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller",
																			"RotationErrorByAccelWeight", NULL);

		this->mc.deadBand = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Controller", "DeadBand", NULL);

		this->mc.failSafeRPMBound = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Controller", "FailSafeRPMBound",
		NULL);
		this->mc.failSafePWMBound = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Controller", "FailSafePWMBound",
		NULL);
		this->mc.failSafeCycles = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Controller", "FailSafeCycles", NULL);

		this->mc.rotationAccelBound = (*sc)["Motion"]->tryGet<double>(0.0, "Motion", "CNMC", "Controller",
																		"MaxRotationAccel");

		this->mc.currentErrorBound = (*sc)["Motion"]->tryGet<short>(0, "Motion", "CNMC", "Controller",
																	"CurrentErrorBound", NULL);
		if (this->mc.currentErrorBound == 0)
		{
			this->mc.controlCurrent = false;
		}
		else
		{
			this->mc.controlCurrent = true;
			this->mc.currentKp = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller", "CurrentKp", NULL);
			this->mc.currentKi = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller", "CurrentKi", NULL);
			this->mc.currentKd = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller", "CurrentKd", NULL);
		}

		this->mc.denominator = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Motors",
															"ThermalMotorConstantDenominator", NULL);
		this->mc.numerator = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Motors", "ThermalMotorConstantNumerator",
		NULL);

		int delta = (int)(((2.0 * M_PI * (double)mc.wheelRadius) / (((double)mc.resolution) * (double)mc.gearReduction))
				* 1000.0);

		this->mc.vmax = (delta * mc.resolution * 2.0 * mc.maxSpeed) / (60.0 * 1000.0);
	}

	void Motion::start()
	{
		if (!Motion::running)
		{
			Motion::running = true;
			this->runThread = thread(run, this);
		}
	}

	/**
	 * Method for checking, whether the Motion's main thread is still running.
	 * @return running
	 */
	bool Motion::isRunning()
	{
		return running;
	}

	void Motion::run()
	{
		// Loop until the driver is closed
		while (running)
		{
			//TODO make the times right!

			// 1 Tick = 100ns, 10 Ticks = 1us
			// remember the time, processing was last triggered
			this->deltaTime = std::chrono::steady_clock::now() - this->cycleLastTimestamp;
			this->cycleLastTimestamp += deltaTime;

			MotionSet* request;

			// Get the next request from the queue
			{
				std::lock_guard<std::mutex> lck(this->motionValueMutex);

				request = this->motionValue;
				this->motionValue = nullptr;
			}

			if (request == null && /*check alive timer*/)
			{
				chrono::milliseconds dura(1);
				this_thread::sleep_for(dura);
				continue;
			}

			if (request != nullptr)
			{// If there is a request, try to process it
				if (running)
				{
					this->executeRequest(request);
				}
				delete request;
			}
			else if ()// check if alive period is over
			{// If there is no request, call the ExecuteCheck method
				if (running)
				{
					this->executeCheck();
				}
			}


			// minCycleTime (us), Ticks (tick), cycleLastTimestamp (tick), 1 tick = 100 ns
			long sleepTime = (this->minCycleTime - (std::chrono::steady_clock::now() - this->cycleLastTimestamp) / 10) / 1000;

			if (sleepTime > 0)
			{
				chrono::milliseconds dura(sleepTime);
				this_thread::sleep_for(dura);
			}

			// Compute the new due time
			long newDueTime = this->alivePeriod - ((std::chrono::steady_clock::now() - this->cycleLastTimestamp) / 10000);

			if (newDueTime <= 0)
			{
				newDueTime = 0;
			}
		}

	}

	/*void Motion::notifyDriverResultAvailable(DriverData* data)
	 {
	 MotionResult* mr = dynamic_cast<MotionResult>(data);
	 if (mr == nullptr)
	 {
	 return;
	 }

	 this.motionValueOld = mr;

	 // Send the raw motion values to the vision
	 msl_actuator_msgs::RawOdometryInfo ro;

	 this->traceModel->trace(new double[] {mr.angle, mr.translation, mr.rotation});

	 ro.position.x = this->traceModel->x;
	 ro.position.y = this->traceModel->y;
	 ro.position.angle = this->traceModel->angle;

	 ro.motion.angle = mr.angle;
	 ro.motion.translation = mr.translation;
	 ro.motion.rotation = mr.rotation;

	 ro.timestamp = (ulong)(DateTime.UtcNow.Ticks - this.odometryDelay);

	 this->rawOdometryInfoPub.publish(ro);

	 // Compute the slip control factor if requested
	 if ((this->slipControlEnabled) && (this->rawMotionValuesOld != nullptr))
	 {

	 // Get the diff angle
	 double adiff = mr.angle - this.rawMotionValuesOld[0];
	 bool slip = false;

	 // Normalize it
	 if (adiff < -M_PI)
	 {
	 adiff += 2.0 * M_PI;
	 }

	 if (adiff > M_PI)
	 {
	 adiff -= 2.0 * M_PI;
	 }

	 if ((Math.Abs(adiff) > this.slipControlDiffAngle)
	 && (Math.Abs(this.rawMotionValuesOld[1]) > this.slipControlDiffAngleMinSpeed))
	 {
	 slip = true;
	 }

	 if ((Math.Abs(rawMotionValuesOld[2]) < this.slipControlOldMaxRot)
	 && (Math.Abs(mr.rotation) > this.slipControlNewMinRot))
	 {
	 slip = true;
	 }

	 // Compute the factor using this equation: factor = 1 / (1 + e^(-2 * factor - x)) where x \in { 0.75, 1.2 }
	 if (slip)
	 {
	 this.slipControlFactor = 1.0 / (1.0 + Math.Exp(-(2.0 * this.slipControlFactor - 1.20)));

	 }
	 else
	 {
	 this.slipControlFactor = 1.0 / (1.0 + Math.Exp(-(3.0 * this.slipControlFactor - 0.75)));
	 }

	 }

	 // Initialize the rawMotionValuesOld cache
	 if (this.rawMotionValuesOld == null)
	 {
	 this.rawMotionValuesOld = new double[3];
	 }

	 // Copy the values
	 this.rawMotionValuesOld[0] = mr.angle;
	 this.rawMotionValuesOld[1] = mr.translation;
	 this.rawMotionValuesOld[2] = mr.rotation;
	 }*/

	/*void Motion::notifyDriverStatusChange(CNMC::StatusCode code, string message)
	 {
	 if (this->quit || ros::ok())
	 {
	 return;
	 }
	 }*/
	void Motion::handleMotionControl(msl_actuator_msgs::MotionControlPtr mc)
	{
		//	TODO: look, if motionValueOld is set and deleted somewhere!
		if (this->accelCompEnabled && this->motionValueOld != nullptr)
		{
			msl_msgs::MotionInfo* m = new msl_msgs::MotionInfo();
			m->angle = this->motionValueOld->angle;
			m->translation = this->motionValueOld->translation;
			m->rotation = this->motionValueOld->rotation;
			mc->motion = *(this->accelComp->compensate(&mc->motion, m));
		}

		{
			std::lock_guard<std::mutex> lck(this->motionValueMutex);
			// Create a new driver command
			if (this->motionValue == nullptr)
				this->motionValue = new MotionSet();
			this->motionValue->angle = mc->motion.angle;
			this->motionValue->translation = mc->motion.translation;
			this->motionValue->rotation = mc->motion.rotation;

			// Apply the slip control if enabled
			if ((this->slipControlEnabled) && (mc->motion.translation > this->slipControlMinSpeed))
			{

				this->motionValue->translation *= this->slipControlFactor;
				this->motionValue->rotation *= this->slipControlFactor;

			}
		}
	}

	/**
	 * This is for handling Strg + C, although no ROS communication was running.
	 * @param sig
	 */
	void Motion::pmSigintHandler(int sig)
	{
		cout << endl << "Motion: Caught SIGINT! Terminating ..." << endl;
		running = false;

		// Call the ros signal handler method
		ros::shutdown();
	}
	/**
	 * This is for handling SIGTERM to terminate
	 * @param sig
	 */
	void Motion::pmSigTermHandler(int sig)
	{
		cout << endl << "Motion: Caught SIGTERM! Terminating ..." << endl;
		running = false;

		// Call the ros signal handler method
		ros::shutdown();
	}

} /* namespace msl_driver */

int main(int argc, char** argv)
{
	msl_driver::Motion* motion = new msl_driver::Motion(argc, argv);
	motion->initCommunication(argc, argv);
	// has to be set after Motion::initCommunication , in order to override the ROS signal handler
	signal(SIGINT, msl_driver::Motion::pmSigintHandler);
	signal(SIGTERM, msl_driver::Motion::pmSigTermHandler);
	motion->initialize();
	motion->open();
	motion->start();

	while (motion->isRunning())
	{
		chrono::milliseconds dura(500);
		this_thread::sleep_for(dura);
	}

	delete motion;
}

