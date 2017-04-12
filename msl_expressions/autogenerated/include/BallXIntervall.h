/*
 * BallXIntervall.h
 *
 *  Created on: May 2, 2016
 *      Author: Stefan Jakob
 */

#pragma once

#include <cnc_geometry/CNPointAllo.h>
#include <engine/USummand.h>
#include <nonstd/optional.hpp>

#include <memory>
#include <string>

namespace alica
{

class BallXIntervall : public USummand
{
  public:
    BallXIntervall(double weight, string name, long id, std::vector<long> relevantEntryPointIds, double minX,
                   double maxX, double tolerance);
    virtual ~BallXIntervall();

    void cacheEvalData();
    virtual UtilityInterval eval(IAssignment *ass);
    string toString();

  private:
    double minX;
    double maxX;
    double tolerance;
    nonstd::optional<geometry::CNPointAllo> alloBall;
};

} /* namespace msl */
