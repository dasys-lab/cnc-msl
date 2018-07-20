#ifndef MSLCONSTRAINTBUILDER_H_
#define MSLCONSTRAINTBUILDER_H_

#include "GeometryCalculator.h"
#include "MSLFootballField.h"
#include "MSLWorldModel.h"
#include "SystemConfig.h"
#include "container/CNPoint2D.h"
#include <AutoDiff.h>
#include <MSLFootballField.h>

#include <memory>
#include <vector>

using namespace std;
using namespace autodiff;

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
    static shared_ptr<Term> lineUpUtil(shared_ptr<geometry::CNPoint2D> norm, double d, vector<shared_ptr<TVec>> &points);
    static shared_ptr<Term> lazyUtil(vector<shared_ptr<TVec>> &robots, vector<shared_ptr<TVec>> &points);

    static shared_ptr<Term> spread(double minDist, vector<shared_ptr<TVec>> &points);

    static shared_ptr<Term> outsideRectangle(shared_ptr<TVec> lowerRightCorner, shared_ptr<TVec> upperLeftCorner, vector<shared_ptr<TVec>> &points);
    static shared_ptr<Term> insideRectangle(shared_ptr<TVec> lowerRightCorner, shared_ptr<TVec> upperLeftCorner, vector<shared_ptr<TVec>> &points);
    static shared_ptr<Term> outsideCorridor(shared_ptr<geometry::CNPoint2D> a, shared_ptr<geometry::CNPoint2D> b, double width,
                                            vector<shared_ptr<TVec>> &points);
    static shared_ptr<Term> insideCorridor(shared_ptr<geometry::CNPoint2D> a, shared_ptr<geometry::CNPoint2D> b, double width,
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
    static shared_ptr<Term> ownPenaltyRules(shared_ptr<TVec> ballT, int executerIdx, vector<shared_ptr<TVec>> &fieldPlayers);
    static shared_ptr<Term> oppPenaltyRules(vector<shared_ptr<TVec>> &fieldPlayers);
    static shared_ptr<Term> ownKickOffRules(shared_ptr<TVec> ballT, int executerIdx, vector<shared_ptr<TVec>> &fieldPlayers);
    static shared_ptr<Term> oppKickOffRules(shared_ptr<TVec> ballT, vector<shared_ptr<TVec>> &fieldPlayers);
    static shared_ptr<Term> ownStdRules(shared_ptr<TVec> ballT, int executerIdx, vector<shared_ptr<TVec>> &fieldPlayers);
    static shared_ptr<Term> oppStdRules(shared_ptr<TVec> ballT, vector<shared_ptr<TVec>> &fieldPlayers);
    static shared_ptr<Term> ownPenaltyAreaDistanceExceptionRule(shared_ptr<TVec> ballT, vector<shared_ptr<TVec>> &fieldPlayers);
    static shared_ptr<Term> ownPenaltyAreaRule(vector<shared_ptr<TVec>> &fieldPlayers);
    static shared_ptr<Term> oppPenaltyAreaRule(vector<shared_ptr<TVec>> &fieldPlayers);

  private:
    static Rules *rules;
    //			static shared_ptr<geometry::CNPoint2D> ownRightSurCornerP;
    //			static shared_ptr<geometry::CNPoint2D> oppLeftSurCornerP;
    //			static shared_ptr<geometry::CNPoint2D> ownRightCornerP;
    //			static shared_ptr<geometry::CNPoint2D> oppLeftCornerP;
    //			static shared_ptr<geometry::CNPoint2D> oppLRHalfP;
    //			static shared_ptr<geometry::CNPoint2D> ownULHalfP;
    //			static shared_ptr<geometry::CNPoint2D> oppLRPenaltyAreaP;
    //			static shared_ptr<geometry::CNPoint2D> oppULPenaltyAreaP;
    //			static shared_ptr<geometry::CNPoint2D> ownLRPenaltyAreaP;
    //			static shared_ptr<geometry::CNPoint2D> ownULPenaltyAreaP;
    //			static shared_ptr<geometry::CNPoint2D> ownLRGoalAreaP;
    //			static shared_ptr<geometry::CNPoint2D> ownULGoalAreaP;
    //			static shared_ptr<geometry::CNPoint2D> oppLRGoalAreaP;
    //			static shared_ptr<geometry::CNPoint2D> oppULGoalAreaP;
    //			static shared_ptr<geometry::CNPoint2D> ownGoalMidP;
    //			static shared_ptr<geometry::CNPoint2D> oppGoalMidP;
    //			static shared_ptr<geometry::CNPoint2D> centreMarkP;

    /*static shared_ptr<TVec> ownRightSurCornerT;
    static shared_ptr<TVec> oppLeftSurCornerT;
    static shared_ptr<TVec> ownRightCornerT;
    static shared_ptr<TVec> oppLeftCornerT;
    static shared_ptr<TVec> oppLRHalfT;
    static shared_ptr<TVec> ownULHalfT;
    static shared_ptr<TVec> oppLRPenaltyAreaT;
    static shared_ptr<TVec> oppULPenaltyAreaT;
    static shared_ptr<TVec> ownLRPenaltyAreaT;
    static shared_ptr<TVec> ownULPenaltyAreaT;
    static shared_ptr<TVec> ownLRGoalAreaT;
    static shared_ptr<TVec> ownULGoalAreaT;
    static shared_ptr<TVec> oppLRGoalAreaT;
    static shared_ptr<TVec> oppULGoalAreaT;
    static shared_ptr<TVec> ownGoalMidT;
    static shared_ptr<TVec> oppGoalMidT;*/
    static shared_ptr<TVec> centreMarkT;

    static void resolveArea(Areas area, shared_ptr<geometry::CNPoint2D> *lowerRightCorner, shared_ptr<geometry::CNPoint2D> *upperLeftCorner);
};

} /* namespace msl */

#endif /* MSLCONSTRAINTBUILDER_H_ */
