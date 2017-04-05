#pragma once

#include "SearchArea.h"

namespace msl
{

class AlloSearchArea : public SearchArea
{
public:
    AlloSearchArea();
    virtual ~AlloSearchArea();
    static shared_ptr<AlloSearchArea> getAnArea(double langle, double hangle, double minDist, double maxDist, geometry::CNPointAllo center,
                                                geometry::CNPositionAllo ownPos);
    shared_ptr<vector<shared_ptr<SearchArea>>> expand();
    bool isValid();
    string toString();

protected:
    AlloSearchArea(double langle, double hangle, double minDist, double maxDist, geometry::CNPointAllo center,
                   geometry::CNPositionAllo ownPos);
    static shared_ptr<vector<shared_ptr<AlloSearchArea>>> daList;
};

} /* namespace msl */
