/*
 * Base.h
 *
 *  Created on: 22.10.2014
 *      Author: Andreas Witsch
 */

#ifndef BASE_H_
#define BASE_H_

#include <iostream>

#include "engine/AlicaEngine.h"
#include "BehaviourCreator.h"
#include "ConditionCreator.h"
#include "UtilityFunctionCreator.h"
#include "ConstraintCreator.h"
#include "MSLWorldModel.h"

using namespace std;

namespace msl
{

	class Base
	{
	public:
		Base(string roleSetName, string masterPlanName, string roleSetDir, bool sim);
		virtual ~Base();

		void start();

		alica::AlicaEngine* ae;
		alica::BehaviourCreator* bc;
		alica::ConditionCreator* cc;
		alica::UtilityFunctionCreator* uc;
		alica::ConstraintCreator* crc;
		MSLWorldModel* wm;
	};

} /* namespace msl */

#endif /* BASE_H_ */
