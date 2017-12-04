#pragma once

#include "Opponents.h"
#include "Teammates.h"

#include <msl_sensor_msgs/SharedWorldInfo.h>
#include <supplementary/InfoBuffer.h>

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
    void processSharedWorldModelData(msl_sensor_msgs::SharedWorldInfoPtr data, supplementary::InfoTime creationTime);

    // Data Access Method
    const supplementary::InfoBuffer<msl_sensor_msgs::SharedWorldInfo> *getSHWMDataBuffer(int robotID) const;
    const std::map<int, shared_ptr<supplementary::InfoBuffer<msl_sensor_msgs::SharedWorldInfo>>>
    getSharedWmDataBuffersMap() const; // TODO: more const?

    Teammates teammates;
    Opponents opponents;

  private:
    MSLWorldModel *wm;
    const supplementary::InfoTime maxValidity = 1000000000;
    std::map<int, std::shared_ptr<supplementary::InfoBuffer<msl_sensor_msgs::SharedWorldInfo>>> shwmDataBuffers;
};

} /* namespace alica */
