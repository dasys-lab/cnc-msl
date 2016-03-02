/*
 * Opponents.cpp
 *
 *  Created on: Feb 26, 2016
 *      Author: Stefan Jakob
 */

#include "Opponents.h"
#include "MSLWorldModel.h"

namespace msl
{

	Opponents::Opponents(MSLWorldModel* wm, int ringBufferLength) : opponentsAlloClustered(ringBufferLength), opponentsEgoClustered(ringBufferLength)
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

	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> Opponents::getOpponentsAlloClustered(int index)
	{
		auto x = opponentsAlloClustered.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	void Opponents::processOpponentsAlloClustered(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opponentsAlloClustered)
	{
		shared_ptr<InformationElement<vector<shared_ptr<geometry::CNPoint2D>>>> o = make_shared<InformationElement<vector<shared_ptr<geometry::CNPoint2D>>>>(
				opponentsAlloClustered, wm->getTime());
		o->certainty = 1;

		this->opponentsAlloClustered.add(o);
	}

	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> Opponents::getOpponentsEgoClustered(int index)
	{
		auto x = opponentsEgoClustered.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	void Opponents::processOpponentsEgoClustered(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opponentsEgoClustered)
	{
		shared_ptr<InformationElement<vector<shared_ptr<geometry::CNPoint2D>>>> o = make_shared<InformationElement<vector<shared_ptr<geometry::CNPoint2D>>>>(
				opponentsEgoClustered, wm->getTime());
		o->certainty = 1;

		this->opponentsEgoClustered.add(o);
	}

} /* namespace msl */


