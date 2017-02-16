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

    const InfoBuffer<geometry::CNPositionAllo> &getTeammatePositionBuffer(int teammateId);
    std::shared_ptr<std::vector<std::shared_ptr<std::pair<int, std::shared_ptr<geometry::CNPositionAllo>>>>> getPositionsOfTeamMates();

    const InfoBuffer<std::vector<std::shared_ptr<geometry::CNPointAllo>>> &getTeammatesAlloClusteredBuffer();
    void integrateTeammatesAlloClustered(std::shared_ptr<std::vector<std::shared_ptr<geometry::CNPointAllo>>> teammatesAlloClustered);

    const InfoBuffer<std::vector<std::shared_ptr<geometry::CNPointEgo>>> &getTeammatesEgoClusteredBuffer();
    void processTeammatesEgoClustered(std::shared_ptr<std::vector<std::shared_ptr<geometry::CNPointEgo>>> teammatesEgoClustered);

    void integrateTeammatesPosition(msl_sensor_msgs::SharedWorldInfoPtr msg, InfoTime creationTime);

  private:
    MSLWorldModel *wm;
    // TODO: replace with ?DEFINES? or whatever for each info type
    const InfoTime maxValidity = 1000000000;
    InfoBuffer<std::vector<std::shared_ptr<geometry::CNPointEgo>>> teammatesEgoClustered;
    InfoBuffer<std::vector<std::shared_ptr<geometry::CNPointAllo>>> teammatesAlloClustered;
    std::map<int, std::shared_ptr<InfoBuffer<geometry::CNPositionAllo>>> robotPositions;
};

} /* namespace msl */
