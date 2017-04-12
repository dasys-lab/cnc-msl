/*
 * DistXContour.h
 *
 *  Created on: Oct 27, 2014
 *      Author: Tobias Schellien
 */

#pragma once

#include <cnc_geometry/CNPointAllo.h>
#include <engine/USummand.h>
#include <nonstd/optional.hpp>

#include <string>
#include <vector>

namespace alica
{

class UtilityInterval;
class IAssignment;

class DistXContour : public USummand
{
  public:
    DistXContour(double weight, string name, long id, vector<long> &relevantEntryPointIds,
                 vector<pair<double, double>> &ContourPoints, double xMaxVal, double xMinVal, int ownId);
    virtual ~DistXContour();
    void cacheEvalData();
    double interpolate2D(double X1, double Y1, double X2, double Y2, double xPoint);
    virtual UtilityInterval eval(IAssignment *ass);

  protected:
    int ownId;
    double weight;
    string name;
    long id;
    std::vector<long> relevantEntryPointIds;
    std::vector<std::pair<double, double>> contourPoints;
    nonstd::optional<geometry::CNPointAllo> alloBall;
    double xMaxVal;
    double xMinVal;
};

} /* namespace alica */
