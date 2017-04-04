/*
 * SearchArea.h
 *
 *  Created on: Feb 4, 2016
 *      Author: Stefan Jakob
 */

#pragma once

using namespace std;

#include <MSLFootballField.h>
#include <cnc_geometry/CNPointAllo.h>
#include <cnc_geometry/CNPositionAllo.h>
#include <cnc_geometry/Calculator.h>
#include <memory>

namespace msl
{

class SearchArea
{
  public:
    virtual ~SearchArea();

    geometry::CNPointAllo midP;
    double langle;
    double hangle;
    double minDist;
    double maxDist;
    double val;
    geometry::CNPointAllo center;
    geometry::CNPositionAllo ownPos;
    // static int compareTo(shared_ptr<SearchArea> a);
    static bool compareTo(shared_ptr<SearchArea> a, shared_ptr<SearchArea> b);
    virtual shared_ptr<vector<shared_ptr<SearchArea>>> expand() = 0;
    virtual bool isValid() = 0;

  protected:
    SearchArea();
    SearchArea(double langle, double hangle, double minDist, double maxDist, geometry::CNPointAllo center,
               geometry::CNPositionAllo ownPos);
    static int counter;
    static int maxNum;
};

} /* namespace msl */
