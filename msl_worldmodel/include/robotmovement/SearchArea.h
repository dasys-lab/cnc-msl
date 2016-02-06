/*
 * SearchArea.h
 *
 *  Created on: Feb 4, 2016
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_SEARCHAREA_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_SEARCHAREA_H_

#include <memory>
#include <GeometryCalculator.h>
#include <MSLFootballField.h>

using namespace std;

namespace msl
{

	class SearchArea
	{
	public:

		virtual ~SearchArea();

		shared_ptr<geometry::CNPoint2D> midP;
		double langle;
		double hangle;
		double minDist;
		double maxDist;
		double val;
		shared_ptr<geometry::CNPoint2D> center;
		shared_ptr<geometry::CNPosition> ownPos;
		int compareTo(shared_ptr<SearchArea> a);
		virtual shared_ptr<vector<shared_ptr<SearchArea>>> expand() = 0;
		virtual bool isValid() = 0;

	protected:
		SearchArea();
		SearchArea(double langle, double hangle, double minDist, double maxDist, shared_ptr<geometry::CNPoint2D> center, shared_ptr<geometry::CNPosition> ownPos);
		MSLFootballField* field = MSLFootballField::getInstance();
		static int counter;
		static int maxNum;
	};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_SEARCHAREA_H_ */
