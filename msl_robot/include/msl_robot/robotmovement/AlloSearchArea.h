/*
 * AlloSearchArea.h
 *
 *  Created on: Feb 4, 2016
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_ALLOSEARCHAREA_H_
#define CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_ALLOSEARCHAREA_H_

#include "SearchArea.h"

namespace msl
{

class AlloSearchArea : public SearchArea
{
  public:
    AlloSearchArea();
    virtual ~AlloSearchArea();
    static shared_ptr<AlloSearchArea> getAnArea(double langle, double hangle, double minDist, double maxDist, shared_ptr<geometry::CNPoint2D> center,
                                                shared_ptr<geometry::CNPosition> ownPos);
    shared_ptr<vector<shared_ptr<SearchArea>>> expand();
    bool isValid();
    string toString();

  protected:
    AlloSearchArea(double langle, double hangle, double minDist, double maxDist, shared_ptr<geometry::CNPoint2D> center,
                   shared_ptr<geometry::CNPosition> ownPos);
    static shared_ptr<vector<shared_ptr<AlloSearchArea>>> daList;
};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_SRC_ROBOTMOVEMENT_ALLOSEARCHAREA_H_ */
