/*
 * DistBallRobot.cpp
 *
 *  Created on: 15.03.2016
 *      Author: endy
 */

#include "DistBallRobot.h"
#include <Ball.h>
#include <Robots.h>
#include <MSLFootballField.h>

using nonstd::optional;

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

		this->sharedBall = wm->ball->getPositionAllo();
		auto opps = wm->robots->opponents.getOpponentsAlloClusteredBuffer().getLastValidContent();

		if (this->sharedBall || !opps)
			return;

		closestOpp.reset();
		double minDist = wm->field->getFieldLength() * 2;
		for (auto& opp : *opps)
		{
			double d = this->sharedBall->distanceTo(opp);
			if (d < 700 && d < minDist)
			{
				minDist = d;
				closestOpp = opp;
			}
		}

		if (closestOpp)
		{
			validAngle = true;
			angleBallOpp = atan2(closestOpp->y - this->sharedBall->y, closestOpp->x - this->sharedBall->x);
		}

		this->teammates = wm->robots->teammates.getPositionsOfTeamMates();
	}

	alica::UtilityInterval DistBallRobot::eval(alica::IAssignment* ass)
	{
		ui.setMin(0.0);
		if (!this->sharedBall)
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

		optional<geometry::CNPositionAllo> curPosition;
		if (relevantRobots != nullptr)
		{
			for (int& robot : *relevantRobots)
			{
				curPosition = this->getPositionOfTeammate(robot);
				if (curPosition == nullptr)
					continue; // This player was not 'positionReceived'

				// SET UI.MIN
				if (!validAngle)
				{
					//if no opp is near ball
					ui.setMin(max(ui.getMin(), 1 - this->sharedBall->distanceTo(curPosition->getPoint()) / wm->field->getMaxDistance()));
				}
				else
				{
					//if an opp is near ball
					double curAngleToBall = atan2(this->sharedBall->y - curPosition->y, this->sharedBall->x - curPosition->x);
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
								(1 - this->sharedBall->distanceTo(curPosition->getPoint()) / wm->field->getMaxDistance()) * scale));
				}
				numAssignedRobots++;
			}
		}

		ui.setMax(ui.getMin());
		// Calculate the best possible robot for this job (ignoring every other summand...)
		if (relevantEntryPoints.at(0)->getMaxCardinality() > numAssignedRobots && ass->getNumUnAssignedRobots() > 0)
		{
			for (int i = 0; i < ass->getNumUnAssignedRobots(); ++i)
			{
				//curPosition = this.playerPositions.GetValue(ass.UnAssignedRobots[i]);
				curPosition = this->getPositionOfTeammate(ass->getUnassignedRobots().at(i));
				if (curPosition == nullptr)
					continue;
				ui.setMax(max(ui.getMax(), 1 - curPosition->distanceTo(*this->sharedBall) / wm->field->getMaxDistance()));
			}
		}
		ui.setMin(max(0.0, ui.getMin()));
		ui.setMax(max(0.0, ui.getMax()));
		return ui;
	}

	optional<geometry::CNPositionAllo> DistBallRobot::getPositionOfTeammate(int robotId)
	{
		if (this->teammates == nullptr)
			return nullptr;

		for (auto& robot : *this->teammates)
		{
			if (robot.first == robotId)
				return robot.second;
		}

		return nullptr;
	}

} /* namespace msl */
