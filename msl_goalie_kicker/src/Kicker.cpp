/*
 * Kicker.cpp
 *
 *  Created on: Aug 19, 2015
 *      Author: Stephan Opfer
 */

#include "Kicker.h"
#include <thread>
#include <signal.h>
#include <SystemConfig.h>
#include <time.h>

namespace msl_driver
{

	bool Kicker::running = false;

	Kicker::Kicker(int argc, char** argv) :
			rosNode(nullptr), spinner(nullptr)
	{
		this->my_serial = nullptr;

		this->sc = supplementary::SystemConfig::getInstance();

		this->ownId = this->sc->getOwnRobotID();

		// Get basic kicker parameters
		string driversPath = (*sc)["Kicker"]->get<string>("Kicker", "Drivers Path", NULL);
		string driverName = (*sc)["Kicker"]->get<string>("Kicker", "Driver", NULL);
		cout << driverName << endl;
		// Read required driver parameters
		this->driverAlivePeriod = (*sc)["Kicker"]->tryGet<int>(250, "Kicker", "Alive Period", NULL);
		this->driverOpenAttemptPeriod = (*sc)["Kicker"]->tryGet<int>(1000, "Kicker", "Open Attempt Period", NULL);

		cout << "Kicker: driver alive period        = " << this->driverAlivePeriod << endl;
		cout << "Kicker: driver open attempt period = " << this->driverOpenAttemptPeriod << endl;
	}

	Kicker::~Kicker()
	{
//		if (this->motionValue != nullptr)
//			delete motionValue;
	}

	/**
	 * Initialises all ROS communcation (sub and pub).
	 */
	void Kicker::initCommunication(int argc, char** argv)
	{
		ros::init(argc, argv, "MSL_Goalie_Kicker");
		rosNode = new ros::NodeHandle();
		spinner = new ros::AsyncSpinner(4);
		handleKickerControlSub = rosNode->subscribe("/KickControl", 10, &Kicker::handleKickControl, (Kicker*)this);
		spinner->start();
	}

	void Kicker::initialize()
	{
//		this->minCycleTime = (*sc)["Kicker"]->tryGet<long>(100000, "Kicker", "Kicker2", "MinCycleTime", NULL);
		this->device = (*sc)["Kicker"]->tryGet<string>("Kicker", "Kicker2", "Device", NULL);

		this->pulseWidthLeft = (*sc)["Kicker"]->get<long>("Kicker", "Kicker2", "Pulse Width Left", NULL);
		this->pulseWidthMiddle = (*sc)["Kicker"]->get<long>("Kicker", "Kicker2", "Pulse Width Middle", NULL);
		this->pulseWidthRight = (*sc)["Kicker"]->get<long>("Kicker", "Kicker2", "Pulse Width Right", NULL);

		this->extensionMaxTime = (*sc)["Kicker"]->tryGet<long>(1000, "Kicker", "Kicker2", "Extension Max Out Time",
		NULL);
		this->extensionMinSleep = (*sc)["Kicker"]->tryGet<long>(4000, "Kicker", "Kicker2", "Extension Min Sleep Time",
		NULL);

		// Read the timeout values
		this->initReadTimeout = (*sc)["Kicker"]->tryGet<int>(1000, "Kicker", "Init Read Timeout", NULL);
		this->readTimeout = (*sc)["Kicker"]->tryGet<int>(3000, "Kicker", "Read Timeout", NULL);
		this->writeTimeout = (*sc)["Kicker"]->tryGet<int>(3000, "Kicker", "Write Timeout", NULL);

		// Output read values
		cout << "Kicker2: device            = " << this->device << endl;
//		cout << "Kicker2: min cycle time    = " << this->minCycleTime << " us" << endl;
		cout << "Kicker2: init read timeout = " << this->initReadTimeout << " ms" << endl;
		cout << "Kicker2: read timeout      = " << this->readTimeout << " ms" << endl;
		cout << "Kicker2: write timeout     = " << this->writeTimeout << " ms" << endl;
	}

	bool Kicker::open()
	{
		// TODO: make baudrate a parameter
		this->my_serial = new serial::Serial(this->device, 57600,
												serial::Timeout::simpleTimeout(this->initReadTimeout));

		cout << "Is the serial port open?";
		if (!my_serial->isOpen())
		{
			cerr << "Kicker: Unable to open serial port : " << this->device << " Errno: " << strerror(errno) << endl;
			return false;
		}

		this->my_serial->setTimeout(serial::Timeout::max(), this->readTimeout, 0, this->writeTimeout, 0);

		//### READY
		this->controllerIsActive = true;

		return true;
	}

	void Kicker::checkSuccess(string name, string command, bool display)
	{
		if (display)
		{
			cout << "Kicker: setting " << name << " (" << command << ")" << endl;
		}

		// Read the answer from the TMC
		string result = readResult(command);

		cout << "Kicker Result: " << result << endl;
	}

	string Kicker::readResult(string command)
	{
		string result = my_serial->readline();

		// Read the echo in case echo is on
		if (result == command)
		{
			result = my_serial->readline();
		}

		// Reset the connection if the controller was reset
		if (result != "" && result.c_str()[0] == '*')
		{
			cout << "Kicker Error: kicker board was reset, reinitializing" << endl;
		}

		return result;
	}

	void Kicker::start()
	{
		// TODO HIER GEHTS WEITER, AB IN NE METHODE ODER SO
//		string maxOut = String.Format("SET Ext Max Out {0}", this.extensionMaxTime);
//		string minSleep = String.Format("SET Ext Min Sleep {0}", this.extensionMinSleep);
//
//		WriteLineKicker(maxOut);
//		CheckSuccess("Extension Max Time", maxOut);
//
//		WriteLineKicker(minSleep);
//		CheckSuccess("Extension Min Sleep Time", minSleep);
//
//
//		string pwl = String.Format("SET Pulse Width Left {0}", this.pulseWidthLeft);
//		string pwm = String.Format("SET Pulse Width Middle {0}", this.pulseWidthMiddle);
//		string pwr = String.Format("SET Pulse Width Right {0}", this.pulseWidthRight);
//
//		WriteLineKicker(pwl);
//		CheckSuccess("pulse width left", pwl);
//
//		WriteLineKicker(pwm);
//		CheckSuccess("pulse width middle", pwm);
//
//		WriteLineKicker(pwr);
//		CheckSuccess("pulse width right", pwr);

		if (!Kicker::running)
		{
			Kicker::running = true;
//			this->runThread = std::thread(&Kicker::run, this);
		}
	}

	/**
	 * Method for checking, whether the Kicker's main thread is still running.
	 * @return running
	 */
	bool Kicker::isRunning()
	{
		return Kicker::running;
	}

	void Kicker::handleKickControl(msl_actuator_msgs::KickControlPtr ks)
	{
		if (Kicker::running == false){
			return;
		}

		std::lock_guard<std::mutex> lck(this->motionValueMutex);

		string kick = string("KICK " + to_string(ks->power));
		string extension = "";
		if (ks->extTime > 0 && ks->extTime < 10000)
		{
			if (ks->extension == 1)
			{
				extension = string("UEXT " + to_string(ks->extTime));
			}
			else if (ks->extension == 2)
			{
				extension = string("REXT " + to_string(ks->extTime));
			}
			else if (ks->extension == 3)
			{
				extension = string("LEXT " + to_string(ks->extTime));
			}
		}

		my_serial->write(kick);
		checkSuccess("kick", kick, false);

		if (extension != "")
		{
			my_serial->write(extension);
			checkSuccess("extension", extension, false);
		}
	}

	/**
	 * This is for handling Strg + C, although no ROS communication was running.
	 * @param sig
	 */
	void Kicker::pmSigintHandler(int sig)
	{
		cout << endl << "Kicker: Caught SIGINT! Terminating ..." << endl;
		Kicker::running = false;

		// Call the ros signal handler method
		ros::shutdown();
	}
	/**
	 * This is for handling SIGTERM to terminate
	 * @param sig
	 */
	void Kicker::pmSigTermHandler(int sig)
	{
		cout << endl << "Kicker: Caught SIGTERM! Terminating ..." << endl;
		Kicker::running = false;

		// Call the ros signal handler method
		ros::shutdown();
	}

} /* namespace msl_driver */

int main(int argc, char** argv)
{
	msl_driver::Kicker* kicker = new msl_driver::Kicker(argc, argv);
	kicker->initCommunication(argc, argv);
// has to be set after Kicker::initCommunication , in order to override the ROS signal handler
	signal(SIGINT, msl_driver::Kicker::pmSigintHandler);
	signal(SIGTERM, msl_driver::Kicker::pmSigTermHandler);
	kicker->initialize();
	bool r = kicker->open();

	if (false == r)
	{
		delete kicker;
		ros::shutdown();

		return 1;
	}

	std::cout << "start" << std::endl;
	kicker->start();
	std::cout << "operating ..." << std::endl;

	while (kicker->isRunning())
	{
		chrono::milliseconds dura(500);
		this_thread::sleep_for(dura);
	}

	delete kicker;
}

