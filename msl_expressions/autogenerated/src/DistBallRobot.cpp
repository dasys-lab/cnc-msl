/*
 * DistBallRobot.cpp
 *
 *  Created on: 15.03.2016
 *      Author: endy
 */

#include "DistBallRobot.h"

namespace msl
{

	DistBallRobot::DistBallRobot(double weight, string name, long id, vector<long> relevantEntryPointIds)
	{
		this->weight = weight;
		this->name = name;
		this->id = id;
		this->relevantEntryPointIds = relevantEntryPointIds;
		wm = MSLWorldModel::get();

		validAngle = false;
		angleBallOpp = 0;
		velAngle = 0;
	}

	DistBallRobot::~DistBallRobot()
	{
		// TODO Auto-generated destructor stub
	}

	void DistBallRobot::cacheEvalData()
	{

	}

	alica::UtilityInterval DistBallRobot::eval(alica::IAssignment* ass)
	{

	}

} /* namespace msl */
