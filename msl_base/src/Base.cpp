/*
 * Base.cpp
 *
 *  Created on: 22.10.2014
 *      Author: endy
 */

#include "Base.h"

namespace msl
{

	Base::Base()
	{
		ae = new alica::AlicaEngine();
		bc = new alicaTests::TestBehaviourCreator();
		cc = new alicaTests::TestConditionCreator();
		uc = new alicaTests::TestUtilityFunctionCreator();
		crc = new alicaTests::TestConstraintCreator();
		ae->setIAlicaClock(new alicaRosProxy::AlicaROSClock());
		ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
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
