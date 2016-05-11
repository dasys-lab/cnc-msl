/*
 * BallXIntervall.cpp
 *
 *  Created on: May 2, 2016
 *      Author: Stefan Jakob
 */

#include <BallXIntervall.h>
#include <MSLWorldModel.h>

namespace alica
{

	BallXIntervall::BallXIntervall(double weight, string name, long id, vector<long> relevantEntryPointIds, double minX, double maxX)
	{
		this->weight = weight;
		this->name = name;
		this->id = id;
		this->minX = minX;
		this->maxX = maxX;
		this->alloBall = nullptr;
		this->halfFieldLength = 0;
		this->relevantEntryPointIds = relevantEntryPointIds;
	}

	BallXIntervall::~BallXIntervall()
	{
	}

	void BallXIntervall::cacheEvalData()
	{
		auto alloBall = msl::MSLWorldModel::get()->ball->getAlloBallPosition();
		if (alloBall != nullptr)
		{
			this->alloBall = alloBall;
		}
		else
		{
			this->alloBall = nullptr;
		}

		this->halfFieldLength = msl::MSLWorldModel::get()->field->getFieldLength() / 2;

	}

	UtilityInterval BallXIntervall::eval(IAssignment* ass)
	{
		this->ui.setMin(0.0);
		this->ui.setMax(0.0);

		if(alloBall == nullptr) {
			this->ui.setMax(0.0);
			return ui;
		}

		double x = alloBall->x;
		if(x >= this->minX && x <= this->maxX)
		{
			this->ui.setMin(1.0);
			this->ui.setMax(1.0);
			return ui;
		}

		if(x <= 0.0)
		{
			this->ui.setMin(-1.0);
			this->ui.setMax(-1.0);
			return ui;
		}

		double val = 0.0;
		if(x > 0 && x < this->minX)
		{
			val = x / this->minX;
		}

		if(x > this->maxX && x < this->halfFieldLength)
		{
			val = 1 - ((x - this->maxX) / (this->halfFieldLength - this->maxX));
		}
		this->ui.setMin(val);
		this->ui.setMax(val);
		return ui;
	}

	string BallXIntervall::toString()
	{
		string retString = this->name + ": ";
		retString += string("W: ") + to_string(this->weight);
		retString += string(" minX: ") + to_string(this->minX);
		retString += string(" maxX: ") + to_string(this->maxX);
		retString += string(" alloBallPos: ") + this->alloBall->toString();
		return retString;
	}

} /* namespace alica */
