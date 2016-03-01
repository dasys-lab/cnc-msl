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

	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> Opponents::getOpponentsAlloClustered()
	{
		return opponentsAlloClustered;
	}

	void Opponents::setOpponentsAlloClustered(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opponentsAlloClustered)
	{
		this->opponentsAlloClustered = opponentsAlloClustered;
	}

	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> Opponents::getOpponentsEgoClustered()
	{
		return opponentsEgoClustered;
	}

	void Opponents::setOpponentsEgoClustered(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opponentsEgoClustered)
	{
		this->opponentsEgoClustered = opponentsEgoClustered;
	}

} /* namespace msl */


