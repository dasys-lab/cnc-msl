#pragma once

#include <SystemConfig.h>
#include <cnc_geometry/CNPointAllo.h>
#include <nonstd/optional.hpp>
#include <supplementary/InfoBuffer.h>
#include <supplementary/InformationElement.h>

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

    const supplementary::InfoBuffer<std::vector<geometry::CNPointAllo>> &getOpponentsAlloClusteredBuffer() const;
    const supplementary::InfoBuffer<std::vector<geometry::CNPointEgo>> &getOpponentsEgoClusteredBuffer() const;

  private:
    MSLWorldModel *wm;
    supplementary::SystemConfig *sc;
    double opponentProtectDistance;
    double opponentProtectAngle;

    const supplementary::InfoTime maxValidity = 1000000000;
    
    supplementary::InfoBuffer<std::vector<geometry::CNPointEgo>> opponentsEgoClustered;
    supplementary::InfoBuffer<std::vector<geometry::CNPointAllo>> opponentsAlloClustered;
};

} /* namespace msl */
