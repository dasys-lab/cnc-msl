/*
 * EgoSearchArea.h
 *
 *  Created on: Feb 4, 2016
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_EGOSEARCHAREA_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_EGOSEARCHAREA_H_

#include "SearchArea.h"

namespace msl
{

class EgoSearchArea : public SearchArea
{
  public:
    EgoSearchArea();
    virtual ~EgoSearchArea();
    void init();
    shared_ptr<EgoSearchArea> getAnArea(double langle, double hangle, double minDist, double maxDist, shared_ptr<geometry::CNPoint2D> center,
                                        shared_ptr<geometry::CNPosition> ownPos);
    shared_ptr<vector<shared_ptr<SearchArea>>> expand();
    bool isValid();
    string toString();
    string toAlloString(shared_ptr<geometry::CNPosition> ownPos);

  protected:
    EgoSearchArea(double langle, double hangle, double minDist, double maxDist, shared_ptr<geometry::CNPoint2D> center,
                  shared_ptr<geometry::CNPosition> ownPos);
    shared_ptr<geometry::CNPoint2D> alloP;
    shared_ptr<vector<shared_ptr<EgoSearchArea>>> daList;
    bool checkAlloPoint(shared_ptr<geometry::CNPoint2D> ap);
};
}
/* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_EGOSEARCHAREA_H_ */
