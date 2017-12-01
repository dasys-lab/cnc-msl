#pragma once

#include "MSLFootballField.h"
#include "MSLWorldModel.h"
#include <AutoDiff.h>
#include <SystemConfig.h>
#include <cnc_geometry/CNPointAllo.h>
#include <cnc_geometry/Calculator.h>
#include <nonstd/optional.hpp>

#include <memory>
#include <vector>

namespace msl
{

enum Areas
{
    Surrounding,
    Field,
    OwnHalf,
    OwnPenaltyArea,
    OwnGoalArea,
    OppHalf,
    OppPenaltyArea,
    OppGoalArea
};

class NoSituationFoundException : std::exception
{
  public:
    Situation situation;
    NoSituationFoundException(Situation situation)
    {
        this->situation = situation;
    }
    virtual const char *what() const throw()
    {
        return "NoSituationFoundException: " + situation;
    }
};

class Rules;

class MSLConstraintBuilder
{
  public:
    static supplementary::SystemConfig *sc;

    static double AREA_TOL;
    static double ON_LINE_TOL;
    static double BLOCK_PASS_WIDTH_TOL;
    static double BLOCK_MIN_RADIUS;
    static double MAX_GOAL_DEFEND_DIST;
    static double MIN_CORRIDOR_WIDTH;
    static double MIN_POSITION_DIST;
    static double MAX_FIELD_DIST;

    static shared_ptr<Term> spreadUtil(vector<shared_ptr<TVec>> &points);
    static shared_ptr<Term> approachUtil(shared_ptr<TVec> destination, vector<shared_ptr<TVec>> &points);
    static shared_ptr<Term> approachUtil(shared_ptr<TVec> destination, shared_ptr<TVec> point);
    static shared_ptr<Term> lineUpUtil(shared_ptr<geometry::CNPointAllo> norm, double d,
                                       vector<shared_ptr<TVec>> &points);
    static shared_ptr<Term> lazyUtil(vector<shared_ptr<TVec>> &robots, vector<shared_ptr<TVec>> &points);

    static shared_ptr<Term> spread(double minDist, vector<shared_ptr<TVec>> &points);

    static shared_ptr<Term> outsideRectangle(shared_ptr<TVec> lowerRightCorner, shared_ptr<TVec> upperLeftCorner,
                                             vector<shared_ptr<TVec>> &points);
    static shared_ptr<Term> insideRectangle(shared_ptr<TVec> lowerRightCorner, shared_ptr<TVec> upperLeftCorner,
                                            vector<shared_ptr<TVec>> &points);
    static shared_ptr<Term> outsideCorridor(shared_ptr<geometry::CNPointAllo> a, shared_ptr<geometry::CNPointAllo> b,
                                            double width, vector<shared_ptr<TVec>> &points);
    static shared_ptr<Term> insideCorridor(geometry::CNPointAllo a, geometry::CNPointAllo b, double width,
                                           vector<shared_ptr<TVec>> &points);

    static shared_ptr<Term> outsideSphere(shared_ptr<TVec> point, double distance, vector<shared_ptr<TVec>> &points);
    static shared_ptr<Term> outsideSphere(shared_ptr<TVec> point, double distance, shared_ptr<TVec> point2);
    static shared_ptr<Term> insideSphere(shared_ptr<TVec> centre, double distance, vector<shared_ptr<TVec>> &points);
    static shared_ptr<Term> insideSphere(shared_ptr<TVec> centre, double distance, shared_ptr<TVec> point);

    static shared_ptr<Term> outsideTriangle(shared_ptr<TVec> a, shared_ptr<TVec> b, shared_ptr<TVec> c, double tolerance, vector<shared_ptr<TVec>> &points);
    static shared_ptr<Term> insideTriangle(shared_ptr<TVec> a, shared_ptr<TVec> b, shared_ptr<TVec> c, double tolerance, vector<shared_ptr<TVec>> &points);
    static shared_ptr<Term> outsideCakePiece(shared_ptr<TVec> a, shared_ptr<TVec> b, shared_ptr<TVec> c, double tolerance, vector<shared_ptr<TVec>> &points);

    static shared_ptr<Term> insideConvex(vector<shared_ptr<TVec>> &shell, double tolerance, vector<shared_ptr<TVec>> &points);
    static shared_ptr<Term> outsideConvex(vector<shared_ptr<TVec>> &shell, double tolerance, vector<shared_ptr<TVec>> &points);
    static shared_ptr<Term> see(shared_ptr<TVec> point, bool considerownPos, double detectionRadius, vector<shared_ptr<TVec>> &points);

    static shared_ptr<Term> outsideArea(Areas area, shared_ptr<TVec> point, double tolerance = AREA_TOL);
    static shared_ptr<Term> outsideArea(Areas area, vector<shared_ptr<TVec>> &points, double tolerance = AREA_TOL);
    static shared_ptr<Term> insideArea(Areas area, shared_ptr<TVec> point, double tolerance = AREA_TOL);
    static shared_ptr<Term> insideArea(Areas area, vector<shared_ptr<TVec>> &points, double tolerance = AREA_TOL);

    static shared_ptr<Term> applyRules(int specialIdx, vector<shared_ptr<TVec>> &fieldPlayers);
    static shared_ptr<Term> applyRules(Situation situation, int specialIdx, vector<shared_ptr<TVec>> &fieldPlayers);

    static shared_ptr<Term> commonRules(vector<shared_ptr<TVec>> &fieldPlayers);
    static shared_ptr<Term> dropBallRules(shared_ptr<TVec> ballT, vector<shared_ptr<TVec>> &fieldPlayers);
    static shared_ptr<Term> ownPenaltyRules(shared_ptr<TVec> ballT, int executerIdx,
                                            vector<shared_ptr<TVec>> &fieldPlayers);
    static shared_ptr<Term> oppPenaltyRules(vector<shared_ptr<TVec>> &fieldPlayers);
    static shared_ptr<Term> ownKickOffRules(shared_ptr<TVec> ballT, int executerIdx,
                                            vector<shared_ptr<TVec>> &fieldPlayers);
    static shared_ptr<Term> oppKickOffRules(shared_ptr<TVec> ballT, vector<shared_ptr<TVec>> &fieldPlayers);
    static shared_ptr<Term> ownStdRules(shared_ptr<TVec> ballT, int executerIdx,
                                        vector<shared_ptr<TVec>> &fieldPlayers);
    static shared_ptr<Term> oppStdRules(shared_ptr<TVec> ballT, vector<shared_ptr<TVec>> &fieldPlayers);
    static shared_ptr<Term> ownPenaltyAreaDistanceExceptionRule(shared_ptr<TVec> ballT,
                                                                vector<shared_ptr<TVec>> &fieldPlayers);
    static shared_ptr<Term> ownPenaltyAreaRule(vector<shared_ptr<TVec>> &fieldPlayers);
    static shared_ptr<Term> oppPenaltyAreaRule(vector<shared_ptr<TVec>> &fieldPlayers);

  private:
    static Rules *rules;
    static shared_ptr<TVec> centreMarkT;

    static void resolveArea(Areas area, geometry::CNPointAllo& lowerRightCorner,
                            geometry::CNPointAllo& upperLeftCorner);
};

} /* namespace msl */
