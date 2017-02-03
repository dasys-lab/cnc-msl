/*
 * Robots.h
 *
 *  Created on: Feb 23, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_ROBOTS_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_ROBOTS_H_

#include "InformationElement.h"
#include "Opponents.h"
#include "RingBuffer.h"
#include "Teammates.h"
#include "msl_sensor_msgs/ObstacleInfo.h"
#include "msl_sensor_msgs/SharedWorldInfo.h"
#include "msl_sensor_msgs/WorldModelData.h"
#include <SystemConfig.h>
#include <container/CNPoint2D.h>
#include <container/CNPosition.h>
#include <map>
#include <vector>

using namespace std;

namespace msl
{

class MSLWorldModel;
class Robots
{
  public:
    Robots(MSLWorldModel *wm, int ringBufferLength);
    virtual ~Robots();
    void processSharedWorldModelData(msl_sensor_msgs::SharedWorldInfoPtr data);
    map<int, shared_ptr<RingBuffer<InformationElement<msl_sensor_msgs::SharedWorldInfo>>>> sharedWolrdModelData;

    shared_ptr<msl_sensor_msgs::SharedWorldInfo> getSHWMData(int robotID, int index = 0);

    Teammates teammates;
    Opponents opponents;

    // TODO implement getRobots => teammates + opponents

  private:
    MSLWorldModel *wm;
    supplementary::SystemConfig *sc;
    unsigned long maxInformationAge = 1000000000;
};

} /* namespace alica */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_ROBOTS_H_ */
