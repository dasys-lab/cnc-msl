#pragma once

#include "InformationElement.h"
#include <cnc_geometry/CNPositionAllo.h>
#include <cnc_geometry/CNPointAllo.h>
#include <InfoBuffer.h>
#include <memory>
#include <vector>

using namespace std;

namespace msl
{

class MSLWorldModel;
class Teammates
{
  public:
    Teammates(MSLWorldModel *wm, int ringBufferLength);
    virtual ~Teammates();
    int teamMatesInOwnPenalty();
    int teamMatesInOppPenalty();

    const InfoBuffer<geometry::CNPositionAllo> *getTeamMatePositionBuffer(int teamMateId) const;
    shared_ptr<vector<shared_ptr<pair<int, shared_ptr<geometry::CNPositionAllo>>>>> getPositionsOfTeamMates();

    const InfoBuffer<vector<shared_ptr<geometry::CNPointAllo>>> &getTeammatesAlloClusteredBuffer();
    void processTeammatesAlloClustered(shared_ptr<vector<shared_ptr<geometry::CNPointAllo>>> teammatesAlloClustered);

    const InfoBuffer<vector<shared_ptr<geometry::CNPointEgo>>> &getTeammatesEgoClusteredBuffer();
    void processTeammatesEgoClustered(shared_ptr<vector<shared_ptr<geometry::CNPointEgo>>> teammatesEgoClustered);

  private:
    MSLWorldModel *wm;
    // TODO: replace with ?DEFINES? or whatever for each info type
    const InfoTime maxValidity = 1000000000;
    InfoBuffer<vector<shared_ptr<geometry::CNPointEgo>>> teammatesEgoClustered;
    InfoBuffer<vector<shared_ptr<geometry::CNPointAllo>>> teammatesAlloClustered;
    map<int, shared_ptr<InfoBuffer<geometry::CNPositionAllo>>> robotPositions;
};

} /* namespace msl */
