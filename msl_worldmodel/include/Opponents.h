#pragma once

#include "InformationElement.h"
#include <SystemConfig.h>
#include <cnc_geometry/CNPointAllo.h>
#include <InfoBuffer.h>
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
    shared_ptr<vector<shared_ptr<geometry::CNPointAllo>>> getOpponentsAlloClustered(int index = 0);
    void processOpponentsAlloClustered(shared_ptr<vector<shared_ptr<geometry::CNPointAllo>>> opponentsAlloClustered);
    shared_ptr<vector<shared_ptr<geometry::CNPointEgo>>> getOpponentsEgoClustered(int index = 0);
    void processOpponentsEgoClustered(shared_ptr<vector<shared_ptr<geometry::CNPointEgo>>> opponentsEgoClustered);
    shared_ptr<geometry::CNPointEgo> getClosestToBall(double &distToOpp);
    shared_ptr<geometry::CNPointEgo> getInCorridor(double angle, double width);

  private:
    MSLWorldModel *wm;
    supplementary::SystemConfig *sc;
    int ringBufferLength;
    double opponentProtectDistance;
    double opponentProtectAngle;
    unsigned long maxInformationAge = 1000000000;
    InfoBuffer<InformationElement<vector<shared_ptr<geometry::CNPointEgo>>>> opponentsEgoClustered;
    InfoBuffer<InformationElement<vector<shared_ptr<geometry::CNPointAllo>>>> opponentsAlloClustered;
};

} /* namespace msl */
