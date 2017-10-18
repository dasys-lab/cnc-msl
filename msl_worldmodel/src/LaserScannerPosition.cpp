/*
 * LaserScannerPosition.cpp
 *
 *  Created on: Oct 11, 2017
 *      Author: cn
 */

#include "LaserScannerPosition.h"
#include "MSLWorldModel.h"
#include <geometry_msgs/Point.h>
#include <container/CNPoint2D.h>

namespace msl
{

LaserScannerPosition::LaserScannerPosition(MSLWorldModel* wm, int ringBufferLength):positions(ringBufferLength)
{
	this->wm = wm;
}

LaserScannerPosition::~LaserScannerPosition()
{
    // TODO Auto-generated destructor stub
}

void LaserScannerPosition::processWorldModelData(geometry_msgs::PointPtr msg)
{
	shared_ptr<geometry::CNPoint2D> opt = make_shared<geometry::CNPoint2D>(msg->x, msg->y);
	        shared_ptr<InformationElement<geometry::CNPoint2D>> o = make_shared<InformationElement<geometry::CNPoint2D>>(
	                opt, wm->getTime());
	        o->certainty = 1;
	        positions.add(o);
}

shared_ptr<geometry::CNPoint2D> LaserScannerPosition::getPosition(int index)
{
	return positions.getLast(index)->getInformation();
}

shared_ptr<geometry::CNPoint2D> LaserScannerPosition::getPosition()
{
	return getPosition(0);
}

} /* namespace msl */
