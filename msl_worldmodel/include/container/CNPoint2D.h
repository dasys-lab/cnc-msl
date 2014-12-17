/*
 * Point2D.h
 *
 *  Created on: 19.11.2014
 *      Author: tobi
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_CONTAINER_POINT2D_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_CONTAINER_POINT2D_H_

#include "geometry_msgs/Point.h"
#include <memory>

using namespace std;


namespace msl {
	class CNPosition;

	class CNPoint2D : public geometry_msgs::Point {
	public:


		CNPoint2D(double x, double y);
		CNPoint2D() : CNPoint2D(0,0) {}

		double length();
		shared_ptr<CNPoint2D> rotate(double radian);
		double angleTo();
		shared_ptr<CNPoint2D> alloToEgo(CNPosition& me);
		shared_ptr<CNPoint2D> egoToAllo(CNPosition& me);

		virtual ~CNPoint2D();
	};
}
#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_CONTAINER_POINT2D_H_ */
