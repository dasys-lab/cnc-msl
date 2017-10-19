#include "DistBallRobot.h"
#include <Ball.h>
#include <Robots.h>
#include <MSLFootballField.h>
#include <container/CNPosition.h>
#include <msl/robot/IntRobotID.h>


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
		validAngle = false;

		sb = wm->ball->getAlloBallPosition();
		auto opps = wm->robots->opponents.getOpponentsAlloClustered();

		if (sb == nullptr || opps == nullptr)
			return;

		closestOpp.reset();
		double minDist = wm->field->getFieldLength() * 2;
		for (auto& opp : *opps)
		{
			double d = sb->distanceTo(opp);
			if (d < 700 && d < minDist)
			{
				minDist = d;
				closestOpp = opp->clone();
			}
		}

		if (closestOpp != nullptr)
		{
			validAngle = true;
			angleBallOpp = atan2(closestOpp->y - sb->y, closestOpp->x - sb->x);
		}

		this->teammates = wm->robots->teammates.getPositionsOfTeamMates();
	}

	alica::UtilityInterval DistBallRobot::eval(alica::IAssignment* ass)
	{
		ui.setMin(0.0);
		if (sb == nullptr)
		{
			//cout << "should not happen" << endl;
			ui.setMax(0.0);
			return ui;
		}
		ui.setMax(1.0);
		int numAssignedRobots = 0;

		// We know that there is only ONE relevant entrypoint for this DistSummand!
		if (relevantEntryPoints.size() == 0)
		{
			cerr << "Wrong Usage of DistBallRobot -> no relevant entrypoints!!!" << endl;
			exit(0);
		}
		auto relevantRobots = ass->getRobotsWorking(relevantEntryPoints.at(0));

		shared_ptr<geometry::CNPosition> curPosition;
		if (relevantRobots != nullptr)
		{
			for (auto robot : *relevantRobots)
			{
				curPosition = this->getPositionOfTeammate(dynamic_cast<const msl::robot::IntRobotID*>(robot));
				if (curPosition == nullptr)
					continue; // This player was not 'positionReceived'

				// SET UI.MIN
				if (!validAngle)
				{
					//if no opp is near ball
					ui.setMin(max(ui.getMin(), 1 - sb->distanceTo(curPosition) / wm->field->getMaxDistance()));
				}
				else
				{
					//if an opp is near ball
					double curAngleToBall = atan2(sb->y - curPosition->y, sb->x - curPosition->x);
					double scale = geometry::deltaAngle(curAngleToBall, angleBallOpp);
					//todo old version differs from the current version as is differently normalized
					/*double scale = abs(angleBallOpp-curAngleToBall);
					 //Normalize
					 while(scale>=M_PI) scale-=M_PI;
					 while(scale<=-M_PI) scale+=M_PI;*/
					scale /= M_PI;
					scale = scale * 0.8 + 0.2;
					scale = 1.0 - scale;

					ui.setMin(
							max(ui.getMin(),
								(1 - sb->distanceTo(curPosition) / wm->field->getMaxDistance()) * scale));
				}
				numAssignedRobots++;
			}
		}

		ui.setMax(ui.getMin());
		// Calculate the best possible robot for this job (ignoring every other summand...)
		if (relevantEntryPoints.at(0)->getMaxCardinality() > numAssignedRobots && ass->getNumUnAssignedRobotIds() > 0)
		{
			for (int i = 0; i < ass->getNumUnAssignedRobotIds(); ++i)
			{
				//curPosition = this.playerPositions.GetValue(ass.UnAssignedRobots[i]);
				curPosition = this->getPositionOfTeammate(dynamic_cast<const msl::robot::IntRobotID*>(ass->getUnassignedRobotIds().at(i)));
				if (curPosition == nullptr)
					continue;
				ui.setMax(max(ui.getMax(), 1 - curPosition->distanceTo(sb) / wm->field->getMaxDistance()));
			}
		}
		ui.setMin(max(0.0, ui.getMin()));
		ui.setMax(max(0.0, ui.getMax()));
		return ui;
	}

	shared_ptr<geometry::CNPosition> DistBallRobot::getPositionOfTeammate(const msl::robot::IntRobotID*  robotId)
	{
		if (this->teammates == nullptr)
			return nullptr;

		for (auto robot : *this->teammates)
		{
			if (robot->first == robotId)
				return robot->second;
		}

		return nullptr;
	}

} /* namespace msl */
