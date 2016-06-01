/*
 * DistXContour.cpp
 *
 *  Created on: Oct 27, 2014
 *      Author: Tobias Schellien
 */

#include <DistXContour.h>
#include "MSLWorldModel.h"
#include <Ball.h>

namespace alica
{

	DistXContour::DistXContour(double weight, string name, long id, vector<long>& relevantEntryPointIds,
								vector<pair<double, double> >& ContourPoints, double xMaxVal, double xMinVal, int ownId)
	{
		this->ownId = ownId;
		this->weight = weight;
		this->name = name;
		this->id = id;
		this->relevantEntryPointIds = relevantEntryPointIds;

		this->xMaxVal = xMaxVal;
		this->xMinVal = xMinVal;
		this->contourPoints = ContourPoints;
		this->alloBall = nullptr;
	}

	DistXContour::~DistXContour()
	{
	}

	void DistXContour::cacheEvalData()
	{
		auto alloBall = msl::MSLWorldModel::get()->ball->getAlloBallPosition();
		if(alloBall != nullptr){
			this->alloBall = alloBall;
		} else {
			this->alloBall = nullptr;
		}

	}

	double DistXContour::interpolate2D(double X1, double Y1, double X2, double Y2, double xPoint)
	{
		return ((Y2 - Y1) / (X2 - X1) * (xPoint - X1) + Y1);
	}

	UtilityInterval DistXContour::eval(IAssignment* ass)
	{
		ui.setMin(0.0);
		ui.setMax(0.0);

		if(alloBall == nullptr) {
			ui.setMax(0.0);
			return ui;
		}

		pair<double, double> lastpoint;
		lastpoint.first = -18000 / 2;
		lastpoint.second = xMinVal;

		pair<double, double> nextpoint;
		nextpoint.first = 18000 / 2;
		nextpoint.second = xMaxVal;

		double val = 0;
		if (this->contourPoints.size() == 0)
		{
			ui.setMax(0.0);
			return ui;
		}
		for (int i = 0; i < this->contourPoints.size(); ++i)
		{
			if (contourPoints[i].first < alloBall->x && contourPoints[i].first > lastpoint.first)
			{
				lastpoint = contourPoints[i];
			}

			if (contourPoints[i].first > alloBall->x && contourPoints[i].first < nextpoint.first)
			{
				nextpoint = contourPoints[i];
			}

		}
		if (alloBall->x > 18000 / 2)
			val = xMaxVal;
		else if (alloBall->x < -18000 / 2)
			val = xMinVal;
		else
			val = interpolate2D(lastpoint.first, lastpoint.second, nextpoint.first, nextpoint.second, alloBall->x);

		ui.setMin(val);
		ui.setMax(val);
		return ui;
	}

} /* namespace alica */
