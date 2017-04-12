/*
 * DistBallRobot.h
 *
 *  Created on: 15.03.2016
 *      Author: endy
 */

#pragma once

#include "MSLWorldModel.h"

#include <engine/IAssignment.h>
#include <engine/USummand.h>
#include <cnc_geometry/CNPointAllo.h>
#include <cnc_geometry/CNPositionAllo.h>
#include <nonstd/optional.hpp>

namespace msl
{

class DistBallRobot : public alica::USummand
{
  public:
    DistBallRobot(double weight, string name, long id, vector<long> relevantEntryPointIds);
    virtual ~DistBallRobot();

    MSLWorldModel *wm;
    nonstd::optional<geometry::CNPointAllo> sharedBall;
    nonstd::optional<geometry::CNPointAllo> closestOpp;
    bool validAngle;
    double angleBallOpp;
    double velAngle;
    std::shared_ptr<std::vector<std::pair<int, geometry::CNPositionAllo>>> teammates;

    virtual void cacheEvalData();

    virtual alica::UtilityInterval eval(alica::IAssignment *ass);
    string toString();
    nonstd::optional<geometry::CNPositionAllo> getPositionOfTeammate(int robotId);
};

} /* namespace msl */
