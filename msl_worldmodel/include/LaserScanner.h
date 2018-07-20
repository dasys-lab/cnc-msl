#pragma once


#include "InformationElement.h"
#include "RingBuffer.h"
#include <msl_sensor_msgs/LaserLocalization.h>

namespace msl {

class MSLWorldModel;
class LaserScanner
{
  public:
    LaserScanner(MSLWorldModel* wm, int ringBufferLength);
    virtual ~LaserScanner();

    void processLaserScannPoints(msl_sensor_msgs::LaserLocalizationPtr msg);
    shared_ptr<msl_sensor_msgs::LaserLocalization> getGoalWallPosition(int index=0);

  private:
    MSLWorldModel* wm;
    int ringBufferlength;
    RingBuffer<InformationElement<msl_sensor_msgs::LaserLocalization>> goalWallPositions;
    unsigned long maxInformationAge = 1000000000;
};

}

