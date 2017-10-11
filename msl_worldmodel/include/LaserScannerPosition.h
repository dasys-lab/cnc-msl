/*
 * LaserScannerPosition.h
 *
 *  Created on: Oct 11, 2017
 *      Author: cn
 */

#ifndef INCLUDE_LASERSCANNERPOSITION_H_
#define INCLUDE_LASERSCANNERPOSITION_H_
#include "container/CNPoint2D.h"
#include <RingBuffer.h>
#include <InformationElement.h>
#include <memory>

namespace msl
{

class MSLWorldModel;

class LaserScannerPosition
{
  public:
    LaserScannerPosition(MSLWorldModel* wm, int ringBufferLength);
    virtual ~LaserScannerPosition();
    void processWorldModelData(geometry_msgs::PointPtr data);
    shared_ptr<geometry::CNPoint2D> getPosition(int index);
  private:
    MSLWorldModel* wm;
    RingBuffer<InformationElement<geometry::CNPoint2D>> positions;
};

} /* namespace msl */

#endif /* INCLUDE_LASERSCANNERPOSITION_H_ */
