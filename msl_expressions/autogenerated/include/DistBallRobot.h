/*
 * DistBallRobot.h
 *
 *  Created on: 15.03.2016
 *      Author: endy
 */

#ifndef DISTBALLROBOT_H_
#define DISTBALLROBOT_H_

#include <engine/USummand.h>
#include <engine/IAssignment.h>
#include "MSLWorldModel.h"

namespace msl
{

	class DistBallRobot : public alica::USummand
	{
	public:
		DistBallRobot(double weight, string name, long id, vector<long> relevantEntryPointIds);
		virtual ~DistBallRobot();


		MSLWorldModel* wm;
		shared_ptr<geometry::CNPoint2D> sb;
		shared_ptr<geometry::CNPoint2D> closestOpp;
		bool validAngle;
		double angleBallOpp;
		double velAngle;
		shared_ptr<vector<shared_ptr<pair<int, shared_ptr<geometry::CNPosition>>>>> teammates;

		virtual void cacheEvalData();

		virtual alica::UtilityInterval eval(alica::IAssignment* ass);
		string toString();
		shared_ptr<geometry::CNPosition> getPositionOfTeammate(int robotId);
	};

} /* namespace msl */

#endif /* DISTBALLROBOT_H_ */
