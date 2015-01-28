/*
 * HaveBall.cpp
 *
 *  Created on: 28.01.2015
 *      Author: tobi
 */

#include "MSLWorldModel.h"
#include "HaveBall.h"
#include "container/CNPoint2D.h"
#include "SystemConfig.h"

namespace msl {

HaveBall::HaveBall(MSLWorldModel* wm) {
	hasBallIteration = 0;
	this->wm = wm;
	sc = SystemConfig::getInstance();

	KICKER_DISTANCE = (*this->sc)["Dribble"]->get<double>("Dribble", "KickerDistance");
	KICKER_ANGLE = M_PI;
	HAVE_BALL_TOLERANCE_DRIBBLE = (*this->sc)["Dribble"]->get<double>("Dribble", "HaveBallToleranceDribble");
	HAVE_BALL_MAX_ANGLE_DELTA = (*this->sc)["Dribble"]->get<double>("Dribble", "HaveBallMaxAngleDelta");
}

HaveBall::~HaveBall() {
	// TODO Auto-generated destructor stub
}


bool HaveBall::operator()() {


		if (!hadBefore) {
			haveBallDistanceDynamic = 0;
		}
		else {
			haveBallDistanceDynamic += 2; //TODO: this won't work, this method is called arbitrarily often in each iteration
			haveBallDistanceDynamic = min(haveBallDistanceDynamic,HAVE_BALL_TOLERANCE_DRIBBLE);
		}

		bool ret = true;
		shared_ptr<CNPoint2D> ballPos = wm->getEgoBallPosition();
		if (ballPos == NULL) return false;

		double angle = ballPos->angleTo();
		double ballDist = ballPos->length();

		double usedKicker;
		if( (2.0 * M_PI/3.0 <= angle) &&  (angle < 4.0 * M_PI/3.0) ){
			if (hadBefore) {
				if (KICKER_DISTANCE+haveBallDistanceDynamic < ballDist) ret=false;
			}
			else if (KICKER_DISTANCE < ballDist) ret=false;
			usedKicker = KICKER_ANGLE;

		}
		//calc angle tolerance to the ball
		double tmpTol = angle - usedKicker;
		//Normalize rotation
		if(tmpTol < -M_PI)
			tmpTol += 2.0*M_PI;
		else if(tmpTol > M_PI)
			tmpTol -= 2.0*M_PI;

		tmpTol = abs( tmpTol );

		if (tmpTol > HAVE_BALL_MAX_ANGLE_DELTA) ret = false;

		return ret;
}


} /* namespace msl */
