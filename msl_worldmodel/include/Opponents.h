#pragma once

#include "InfoBuffer.h"
#include "InformationElement.h"

#include <SystemConfig.h>
#include <cnc_geometry/CNPointAllo.h>
#include <nonstd/optional.hpp>

#include <memory>
#include <vector>

namespace msl
{

class MSLWorldModel;
class Opponents
{
  public:
    Opponents(MSLWorldModel *wm, int ringBufferLength);
    virtual ~Opponents();
    double getOpponentProtectDistance();
    double getOpponentProtectAngle();

    void integrateOpponentsAlloClustered(std::shared_ptr<const std::vector<geometry::CNPointAllo>> opponentsAlloClustered);
    void integrateOpponentsEgoClustered(std::shared_ptr<const std::vector<geometry::CNPointEgo>> opponentsEgoClustered);

    nonstd::optional<geometry::CNPointEgo> getClosestToBall(double &distToOpp);
    nonstd::optional<geometry::CNPointEgo> getInCorridor(double angle, double width);

    const InfoBuffer<std::vector<geometry::CNPointAllo>> &getOpponentsAlloClusteredBuffer() const;
    const InfoBuffer<std::vector<geometry::CNPointEgo>> &getOpponentsEgoClusteredBuffer() const;

  private:
    MSLWorldModel *wm;
    supplementary::SystemConfig *sc;
    double opponentProtectDistance;
    double opponentProtectAngle;

    const InfoTime maxValidity = 1000000000;
    
    InfoBuffer<std::vector<geometry::CNPointEgo>> opponentsEgoClustered;
    InfoBuffer<std::vector<geometry::CNPointAllo>> opponentsAlloClustered;
};

} /* namespace msl */
