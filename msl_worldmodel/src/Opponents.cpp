/*
 * Opponents.cpp
 *
 *  Created on: Feb 26, 2016
 *      Author: Stefan Jakob
 */

#include "Opponents.h"

namespace msl
{

	Opponents::Opponents(MSLWorldModel* wm, int ringBufferLength)
	{
		this->wm = wm;
		this->sc = supplementary::SystemConfig::getInstance();
		this->ringBufferLength = ringBufferLength;
		this->opponentProtectAngle = (*sc)["WorldModel"]->get<double>("WorldModel.OpponentProtectAngle", NULL);
		this->opponentProtectDistance = (*sc)["WorldModel"]->get<double>("WorldModel.OpponentProtectDistance", NULL);
	}

	Opponents::~Opponents()
	{
	}

	double Opponents::getOpponentProtectDistance()
	{
		return this->opponentProtectDistance;
	}

	double Opponents::getOpponentProtectAngle()
	{
		return this->opponentProtectAngle;
	}

} /* namespace msl */
