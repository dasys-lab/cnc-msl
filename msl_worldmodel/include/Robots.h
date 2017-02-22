#pragma once

#include "InfoBuffer.h"
#include "Opponents.h"
#include "Teammates.h"

#include <msl_sensor_msgs/SharedWorldInfo.h>

#include <map>
#include <memory>

namespace msl
{

class MSLWorldModel;
class Robots
{
  public:
    Robots(MSLWorldModel *wm, int ringBufferLength);
    virtual ~Robots();

    // Data Integration Method
    void processSharedWorldModelData(msl_sensor_msgs::SharedWorldInfoPtr data, InfoTime creationTime);

    // Data Access Method
    const InfoBuffer<msl_sensor_msgs::SharedWorldInfo> *getSHWMDataBuffer(int robotID) const;
    const std::map<int, shared_ptr<InfoBuffer<msl_sensor_msgs::SharedWorldInfo>>>
    getSharedWmDataBuffersMap() const; // TODO: more const?

    Teammates teammates;
    Opponents opponents;

  private:
    MSLWorldModel *wm;
    const InfoTime maxValidity = 1000000000;
    std::map<int, std::shared_ptr<InfoBuffer<msl_sensor_msgs::SharedWorldInfo>>> shwmDataBuffers;
};

} /* namespace alica */
