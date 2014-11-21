/*
 * GeometryTransformer.cpp
 *
 *  Created on: 15.11.2014
 *      Author: tobi
 */

#include "GeometryTransformer.h"



namespace msl {


	GeometryTransformer::GeometryTransformer() {
		// TODO Auto-generated constructor stub

	}

	pair<double, double> GeometryTransformer::allo2Ego(pair<double, double>& p, tuple<double, double, double>& ownPos)
		{
		  pair<double, double> ego;

		  double x = p.first - std::get<0>(ownPos);
		  double y = p.second - std::get<1>(ownPos);

		  double angle = atan2(y, x) - std::get<2>(ownPos);
		  double dist = sqrt(x * x + y * y);

		  ego.first = cos(angle) * dist;
		  ego.second = sin(angle) * dist;

		  return ego;
		}


} /* namespace msl */
