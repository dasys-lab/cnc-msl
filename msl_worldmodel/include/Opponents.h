/*
 * Opponents.h
 *
 *  Created on: Feb 26, 2016
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_OPPONENTS_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_OPPONENTS_H_

#include "InformationElement.h"
#include "RingBuffer.h"
#include <SystemConfig.h>
#include <container/CNPoint2D.h>
#include <vector>

using namespace std;

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
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> getOpponentsAlloClustered(int index = 0);
    void processOpponentsAlloClustered(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opponentsAlloClustered);
    shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> getOpponentsEgoClustered(int index = 0);
    void processOpponentsEgoClustered(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opponentsEgoClustered);
    shared_ptr<geometry::CNPoint2D> getClosestToBall(double &distToOpp);
    shared_ptr<geometry::CNPoint2D> getInCorridor(double angle, double width);

  private:
    MSLWorldModel *wm;
    supplementary::SystemConfig *sc;
    int ringBufferLength;
    double opponentProtectDistance;
    double opponentProtectAngle;
    unsigned long maxInformationAge = 1000000000;
    RingBuffer<InformationElement<vector<shared_ptr<geometry::CNPoint2D>>>> opponentsEgoClustered;
    RingBuffer<InformationElement<vector<shared_ptr<geometry::CNPoint2D>>>> opponentsAlloClustered;
};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_OPPONENTS_H_ */
