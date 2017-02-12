#pragma once

#include "InformationElement.h"
#include "RingBuffer.h"
#include <cnc_geometry/CNPositionAllo.h>
#include <cnc_geometry/CNPointAllo.h>
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
    shared_ptr<geometry::CNPosition> getTeamMatePosition(int teamMateId, int index = 0);
    shared_ptr<vector<shared_ptr<pair<int, shared_ptr<geometry::CNPosition>>>>> getPositionsOfTeamMates();
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> getTeammatesAlloClustered(int index = 0);
    void processTeammatesAlloClustered(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> teammatesAlloClustered);
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> getTeammatesEgoClustered(int index = 0);
    void processTeammatesEgoClustered(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> teammatesEgoClustered);

    map<int, shared_ptr<RingBuffer<InformationElement<geometry::CNPosition>>>> robotPositions;

  private:
    MSLWorldModel *wm;
    int ringBufferLength;
    unsigned long maxInformationAge = 1000000000;
    RingBuffer<InformationElement<vector<shared_ptr<geometry::CNPoint2D>>>> teammatesEgoClustered;
    RingBuffer<InformationElement<vector<shared_ptr<geometry::CNPoint2D>>>> teammatesAlloClustered;
};

} /* namespace msl */
