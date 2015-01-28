/*
 * HaveBall.h
 *
 *  Created on: 28.01.2015
 *      Author: tobi
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_HAVEBALL_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_HAVEBALL_H_

#include "SystemConfig.h"

using namespace supplementary;

namespace msl {
class MSLWorldModel;

class HaveBall {
public:
	HaveBall(MSLWorldModel* wm);
	virtual ~HaveBall();
	bool operator()();
	double HAVE_BALL_TOLERANCE_DRIBBLE;
	double KICKER_DISTANCE;
	double KICKER_ANGLE;
	double HAVE_BALL_MAX_ANGLE_DELTA;
private:
	int hasBallIteration;
	bool hadBefore;
	double haveBallDistanceDynamic;
	MSLWorldModel* wm;
	SystemConfig* sc;
};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_HAVEBALL_H_ */
