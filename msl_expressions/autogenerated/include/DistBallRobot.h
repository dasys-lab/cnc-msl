#pragma once

#include <engine/USummand.h>
#include <engine/IAssignment.h>
#include "MSLWorldModel.h"

namespace supplementary{
	class IAgentID;
}

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
		shared_ptr<vector<shared_ptr<pair< const supplementary::IAgentID*, shared_ptr<geometry::CNPosition>>>>> teammates;

		virtual void cacheEvalData();

		virtual alica::UtilityInterval eval(alica::IAssignment* ass);
		string toString();
		shared_ptr<geometry::CNPosition> getPositionOfTeammate(const supplementary::IAgentID*  robotId);
	};

} /* namespace msl */
