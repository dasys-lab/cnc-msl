/*
 * Base.cpp
 *
 *  Created on: 22.10.2014
 *      Author: endy
 */

#include <iostream>

#include <thread>
#include <chrono>
#include "ros/ros.h"
#include "Base.h"
#include "clock/AlicaROSClock.h"
#include "communication/AlicaRosCommunication.h"
#include "SigFault.h"
#include "robotmovement/RobotMovement.h"

using namespace std;
using namespace msl;

namespace msl
{

	Base::Base(string roleSetName, string masterPlanName, string roleSetDir)
	{
		ae = new alica::AlicaEngine();
		bc = new alica::BehaviourCreator();
		cc = new alica::ConditionCreator();
		uc = new alica::UtilityFunctionCreator();
		crc = new alica::ConstraintCreator();
		ae->setIAlicaClock(new alicaRosProxy::AlicaROSClock());
		ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));

		wm = MSLWorldModel::get();

		RobotMovement::readConfigParameters();

		ae->init(bc, cc, uc, crc, roleSetName, masterPlanName, roleSetDir, false);
	}

	void Base::start()
	{
		ae->start();
	}

	Base::~Base()
	{
		ae->shutdown();
		delete ae->getIAlicaClock();
		delete ae->getCommunicator();
		delete ae;
		delete cc;
		delete bc;
		delete uc;
		delete crc;
	}

} /* namespace msl */

int main(int argc, char** argv)
{
	cout << "Initing Ros" << endl;
	ros::init(argc, argv, "AlicaEngine");

	//This makes segfaults to exceptions
	segfaultdebug::init_segfault_exceptions();

	cout << "Starting Base" << endl;
	if (argc < 2)
	{
		cout << "Usage: Base -m [Masterplan] -rd [rolesetdir]" << endl;
		return 0;
	}

	string masterplan = "";
	string rolesetdir = "";
	string roleset = "";

	for (int i = 1; i < argc; i++)
	{
		if (string(argv[i]) == "-m" || string(argv[i]) == "-masterplan")
		{
			masterplan = argv[i + 1];
			i++;
		}

		if (string(argv[i]) == "-rd" || string(argv[i]) == "-rolesetdir")
		{
			rolesetdir = argv[i + 1];
			i++;
		}
		if (string(argv[i]) == "-r" || string(argv[i]) == "-roleset")
		{
			roleset = argv[i + 1];
			i++;
		}
	}
	if (masterplan.size() == 0 || rolesetdir.size() == 0)
	{
		cout << "Usage: Base -m [Masterplan] -rd [rolesetdir]" << endl;
		return 0;
	}
	cout << "Masterplan is: " << masterplan << endl;
	cout << "Rolset Directory is: " << rolesetdir << endl;
	cout << "Rolset is: " << roleset << endl;

	cout << "Constructing Base" << endl;
	Base* base = new Base(roleset, masterplan, rolesetdir);

	base->start();

	while (ros::ok())
	{
		std::chrono::milliseconds dura(500);
		std::this_thread::sleep_for(dura);
	}

	return 0;
}
