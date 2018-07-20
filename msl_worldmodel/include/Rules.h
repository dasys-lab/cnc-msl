/*
 * Rules.h
 *
 *  Created on: Aug 27, 2015
 *      Author: labpc1
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_RULES_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_RULES_H_

#include "SystemConfig.h"

using namespace supplementary;

using namespace std;

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
    SystemConfig *sc;

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

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_RULES_H_ */
