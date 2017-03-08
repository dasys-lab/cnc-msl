/*
 * Rules.h
 *
 *  Created on: Aug 27, 2015
 *      Author: labpc1
 */

#pragma once

#include <SystemConfig.h>

namespace msl
{

class Rules
{
  public:
    static Rules *getInstance();
    double getBallRadius();
    double getRobotRadius();
    unsigned long getStandbyTime();
    double getStayAwayRadius();
    double getStayAwayRadiusOpp();
    double getStayAwayRadiusDropBall();
    double getPushDistance();
    double getKickDistance();
    unsigned long getPenaltyTimeForShot();

  private:
    supplementary::SystemConfig *sc;

    double ballRadius;
    double robotRadius;
    unsigned long standbyTime;
    double stayAwayRadius;
    double stayAwayRadiusOpp;
    double stayAwayRadiusDropBall;
    double pushDistance;
    double kickDistance;
    unsigned long penaltyTimeForShot;

    Rules();
    virtual ~Rules();
};

} /* namespace msl */
