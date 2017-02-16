#pragma once

#include "InformationElement.h"
#include "InfoBuffer.h"

#include <SystemConfig.h>
#include <cnc_geometry/CNPointAllo.h>

#include <vector>
#include <memory>

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

    void processOpponentsAlloClustered(shared_ptr<vector<shared_ptr<geometry::CNPointAllo>>> opponentsAlloClustered);
    shared_ptr<vector<shared_ptr<geometry::CNPointEgo>>> getOpponentsEgoClustered(int index = 0);
    void processOpponentsEgoClustered(shared_ptr<vector<shared_ptr<geometry::CNPointEgo>>> opponentsEgoClustered);
    shared_ptr<geometry::CNPointEgo> getClosestToBall(double &distToOpp);
    shared_ptr<geometry::CNPointEgo> getInCorridor(double angle, double width);

    const InfoBuffer<vector<shared_ptr<geometry::CNPointAllo>>> &getOpponentsAlloClusteredBuffer() const;
    const InfoBuffer<vector<shared_ptr<geometry::CNPointEgo>>> &getOpponentsEgoClusteredBuffer() const;

  private:
    MSLWorldModel *wm;
    supplementary::SystemConfig *sc;
    double opponentProtectDistance;
    double opponentProtectAngle;

    const InfoTime maxValidity = 1000000000;
    InfoBuffer<vector<shared_ptr<geometry::CNPointEgo>>> opponentsEgoClustered;
    InfoBuffer<vector<shared_ptr<geometry::CNPointAllo>>> opponentsAlloClustered;
};

} /* namespace msl */
