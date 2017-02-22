#pragma once

#include "InfoBuffer.h"

#include <cnc_geometry/CNPositionAllo.h>
#include <cnc_geometry/CNPointAllo.h>

#include <memory>
#include <vector>
#include <map>

namespace msl_sensor_msgs {
	ROS_DECLARE_MESSAGE(SharedWorldInfo);
}

namespace msl
{

class MSLWorldModel;
class Teammates
{
  public:
    Teammates(MSLWorldModel *wm, int ringBufferLength);
    virtual ~Teammates();
    int teammatesInOwnPenalty();
    int teammatesInOppPenalty();

    void integrateTeammatesAlloClustered(std::shared_ptr<std::vector<geometry::CNPointAllo>> teammatesAlloClustered);
    void integrateTeammatesEgoClustered(std::shared_ptr<std::vector<geometry::CNPointEgo>> teammatesEgoClustered);
    void integrateTeammatesPosition(msl_sensor_msgs::SharedWorldInfoPtr msg, InfoTime creationTime);

    const InfoBuffer<std::vector<geometry::CNPointAllo>> &getTeammatesAlloClusteredBuffer();
    const InfoBuffer<std::vector<geometry::CNPointEgo>> &getTeammatesEgoClusteredBuffer();

    const InfoBuffer<geometry::CNPositionAllo> &getTeammatePositionBuffer(int teammateId);

    std::shared_ptr<std::vector<std::pair<int, geometry::CNPositionAllo>>> getPositionsOfTeamMates();

  private:
    MSLWorldModel *wm;
    // TODO: replace with ?DEFINES? or whatever for each info type
    const InfoTime maxValidity = 1000000000;

    InfoBuffer<std::vector<geometry::CNPointEgo>> teammatesEgoClustered;
    InfoBuffer<std::vector<geometry::CNPointAllo>> teammatesAlloClustered;

    std::map<int, std::shared_ptr<InfoBuffer<geometry::CNPositionAllo>>> robotPositions;
};

} /* namespace msl */
