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
    shared_ptr<geometry::CNPositionAllo> getTeamMatePosition(int teamMateId, int index = 0);
    shared_ptr<vector<shared_ptr<pair<int, shared_ptr<geometry::CNPositionAllo>>>>> getPositionsOfTeamMates();
    shared_ptr<vector<shared_ptr<geometry::CNPointAllo>>> getTeammatesAlloClustered(int index = 0);
    void processTeammatesAlloClustered(shared_ptr<vector<shared_ptr<geometry::CNPointAllo>>> teammatesAlloClustered);
    shared_ptr<vector<shared_ptr<geometry::CNPointEgo>>> getTeammatesEgoClustered(int index = 0);
    void processTeammatesEgoClustered(shared_ptr<vector<shared_ptr<geometry::CNPointEgo>>> teammatesEgoClustered);

    map<int, shared_ptr<InfoBuffer<InformationElement<geometry::CNPositionAllo>>>> robotPositions;

  private:
    MSLWorldModel *wm;
    int ringBufferLength;
    unsigned long maxInformationAge = 1000000000;
    InfoBuffer<InformationElement<vector<shared_ptr<geometry::CNPointEgo>>>> teammatesEgoClustered;
    InfoBuffer<InformationElement<vector<shared_ptr<geometry::CNPointAllo>>>> teammatesAlloClustered;
};

} /* namespace msl */
