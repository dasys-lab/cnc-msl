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
		this->driverAlivePeriod = (*sc)["Kicker"]->get<int>("Kicker", "Alive Period", NULL);
		this->driverOpenAttemptPeriod = (*sc)["Kicker"]->get<int>("Kicker", "Open Attempt Period", NULL);

		cout << "Kicker: driver alive period        = " << this->driverAlivePeriod << endl;
		cout << "Kicker: driver open attempt period = " << this->driverOpenAttemptPeriod << endl;
	}

	Kicker::~Kicker()
	{
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
		this->device = (*sc)["Kicker"]->get<string>("Kicker", "Kicker2", "Device", NULL);

		this->pulseWidthLeft = (*sc)["Kicker"]->get<long>("Kicker", "Kicker2", "Pulse Width Left", NULL);
		this->pulseWidthMiddle = (*sc)["Kicker"]->get<long>("Kicker", "Kicker2", "Pulse Width Middle", NULL);
		this->pulseWidthRight = (*sc)["Kicker"]->get<long>("Kicker", "Kicker2", "Pulse Width Right", NULL);

		this->extensionMaxTime = (*sc)["Kicker"]->get<long>("Kicker", "Kicker2", "Extension Max Out Time",
		NULL);
		this->extensionMinSleep = (*sc)["Kicker"]->get<long>("Kicker", "Kicker2", "Extension Min Sleep Time",
		NULL);

		// Read the timeout values
		this->initReadTimeout = (*sc)["Kicker"]->get<int>("Kicker", "Init Read Timeout", NULL);
		this->readTimeout = (*sc)["Kicker"]->get<int>("Kicker", "Read Timeout", NULL);
		this->writeTimeout = (*sc)["Kicker"]->get<int>("Kicker", "Write Timeout", NULL);

		// Output read values
		cout << "Kicker2: device            = " << this->device << endl;
		cout << "Kicker2: init read timeout = " << this->initReadTimeout << " ms" << endl;
		cout << "Kicker2: read timeout      = " << this->readTimeout << " ms" << endl;
		cout << "Kicker2: write timeout     = " << this->writeTimeout << " ms" << endl;
	}

	bool Kicker::open()
	{
		// TODO: make baudrate a parameter
		this->my_serial = new serial::Serial(this->device, 57600,
												serial::Timeout::simpleTimeout(this->initReadTimeout));

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

	bool Kicker::checkSuccess(string name, string command, bool display)
	{
		if (display)
		{
			cout << "Kicker: setting " << name << " (" << command << ")" << endl;
		}

		// Read the answer from the TMC
		string result = readResult(command + "\n");

		cout << "Kicker Result: " << result;
	
		if(result != "OK\n")
			return false;

		return true;
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
		bool result = true;
		result &= this->writeAndCheck("SET Ext Max Out " + to_string(this->extensionMaxTime));
		result &= this->writeAndCheck("SET Ext Min Sleep " + to_string(this->extensionMinSleep));
		
		result &= this->writeAndCheck("SET Pulse Width Left " + to_string(this->pulseWidthLeft));
		result &= this->writeAndCheck("SET Pulse Width Middle " + to_string(this->pulseWidthMiddle)); 
		result &= this->writeAndCheck("SET Pulse Width Right " + to_string(this->pulseWidthRight));

		if (result == false)
		{
			Kicker::running = false;
		} 
		else if (!Kicker::running)
		{
			Kicker::running = true;
		}
	}

	bool Kicker::writeAndCheck(string cmd)
	{	
		this->my_serial->write(cmd + "\n");
                return this->checkSuccess(cmd, cmd, true);
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
		std::cout << "Received Mesg" << std::endl;
		if (Kicker::running == false){
			std::cout << "Running is false" << std::endl;
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

		writeAndCheck(kick);

		if (extension != "")
		{
			writeAndCheck(extension);
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

