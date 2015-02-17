/*
 * GeometryTransformer.cpp
 *
 *  Created on: 15.11.2014
 *      Author: tobi
 */

#include <GeometryCalculator.h>



namespace msl {

	double GeometryCalculator::deltaAngle(double angle1, double angle2) {
		double ret = angle2 - angle1;
			if (ret > M_PI)
				return -M_2_PI + ret;
			else if (ret < -M_PI)
				return ret + M_2_PI;
			else
				return ret;

	}

	GeometryCalculator::GeometryCalculator() {

	}

//	pair<double, double> GeometryCalculator::allo2Ego(pair<double, double>& p, tuple<double, double, double>& ownPos)
//		{
//		  pair<double, double> ego;
//
//		  double x = p.first - std::get<0>(ownPos);
//		  double y = p.second - std::get<1>(ownPos);
//
//		  double angle = atan2(y, x) - std::get<2>(ownPos);
//		  double dist = sqrt(x * x + y * y);
//
//		  ego.first = cos(angle) * dist;
//		  ego.second = sin(angle) * dist;
//
//		  return ego;
//		}


} /* namespace msl */
