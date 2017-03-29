#ifndef MSLFootballField_H
#define MSLFootballField_H

#include <GeometryCalculator.h>
#include <SystemConfig.h>

//                   T  H  E    E  N  E  M  Y  `  S    S  I  D  E
//
// 30---------------------------------------------------------------------------+
//|                              .---------------.                             |
//|                Sur-          |    Opp Goal   |       rounding              |
//|                              |               |                             |
//|      3----------------22-26--7------15-------8-----------------------4     |
//|      |                |  |     Opp Goal Area     |  |                |     |
//|      |                |  '----------------------25  |                |     |
//|      |                |       Opp Penalty Area      |                |     |
//|      |                '----------------------------21                |     |
//|      |                                                               |     |
//|      |            14                 12                13            |     |
//|      |                                                               |     |
//|      |                            Opp Half                           |     |
//|      |                                                               |     |
//|  S   |                                 X                             |  S  |
//|  U   |                               ^                               |  U  |
//|  R   |                               |                               |  R  |
//|  R   |                            ,--|--,                            |  R  |
//|  O   |                     Y     /   |   \                           |  O  |
//|  U   18-----------27--------<<-------0----|------------28-----------17  U  |
//|  N   |                           \       /                           |  N  |
//|  D   |                            `-----´                            |  D  |
//|  I   |                                                               |  I  |
//|  N   |                                                               |  N  |
//|  G   |                            Own Half                           |  G  |
//|      |                                                               |     |
//|      |                                                               |     |
//|      |            11                 9                 10            |     |
//|      |                                                               |     |
//|      |                20----------------------------.                |     |
//|      |                |       Own Penalty Area      |                |     |
//|      |                |  24----------------------.  |                |     |
//|      |                |  |      Own Goal Area    |  |                |     ^ X ^
//|      1-----------------------5-------16------6--23-19----------------2     |   H
//|                              |               |                             |   E
//|              Sur-            |    Own Goal   |       rounding              |   I
//|                              '---------------'                             |   G
//+-------------------------------------------------------------------<-------29   H
//                                                                    Y            T
//                             O  U  R    S  I  D  E                 < W I D T H > v
//
// OppGoal angle 0°
// turning left/counter-clock wise -> positive angle
// turning right/clock wise -> negative angle
//
// Point2D 0    = PosCenterMarker
// Point2D 1    = PosLeftOwnFieldPost
// Point2D 2    = PosRightOwnFieldPost
// Point2D 3    = PosLeftOppFieldPost
// Point2D 4    = PosRightOppFieldPost
// Point2D 5    = PosLeftOwnGoalPost
// Point2D 6    = PosRightOwnGoalPost
// Point2D 7    = PosLeftOppGoalPost
// Point2D 8    = PosRightOppGoalPost
// Point2D 9    = PosOwnPenaltyMarker
// Point2D 10   = PosRightOwnRestartMarker
// Point2D 11   = PosLeftOwnRestartMarker
// Point2D 12   = PosOpponentPenaltyMarker
// Point2D 13   = PosRightOppRestartMarker
// Point2D 14   = PosLeftOppRestartMarker
// Point2D 15   = PosOppGoalMid
// Point2D 16   = PosOwnGoalMid
// LR = lower right corner, UL = upper left corner
// Point2D 17   = PosLROppHalf
// Point2D 18   = PosULOwnHalf
// Point2D 19   = PosLROwnPenaltyArea
// Point2D 20   = PosULOwnPenaltyArea
// Point2D 21   = PosLROppPenaltyArea
// Point2D 22   = PosULOppPenaltyArea
// Point2D 23   = PosLROwnGoalArea
// Point2D 24   = PosULOwnGoalArea
// Point2D 25   = PosLROppGoalArea
// Point2D 26   = PosULOppGoalArea
// Point2D 27   = PosLeftRestartMarker
// Point2D 28   = PosRightRestartMarker
// Point2D 29   = PosLRSurrounding
// Point2D 30   = PosULSurrounding

namespace msl
{
class MSLWorldModel;
class MSLFootballField
{

  public:
    MSLFootballField(MSLWorldModel *wm);
    virtual ~MSLFootballField();

    bool isInsideField(shared_ptr<geometry::CNPoint2D> point, double tolerance = 0);
    bool isInsideField(double x, double y, double tolerance = 0);

    bool isInsidePenalty(shared_ptr<geometry::CNPoint2D> p, double tolerance);
    bool isInsideOwnPenalty(shared_ptr<geometry::CNPoint2D> p, double tolerance);
    bool isInsideOppPenalty(shared_ptr<geometry::CNPoint2D> p, double tolerance);

    bool isInsideGoalArea(shared_ptr<geometry::CNPoint2D> p, double tolerance);
    bool isInsideOwnGoalArea(shared_ptr<geometry::CNPoint2D> p, double tolerance);
    bool isInsideOppGoalArea(shared_ptr<geometry::CNPoint2D> p, double tolerance);

    shared_ptr<geometry::CNPoint2D> mapOutOfPenalty(shared_ptr<geometry::CNPoint2D> inp);
    shared_ptr<geometry::CNPoint2D> mapOutOfOwnPenalty(shared_ptr<geometry::CNPoint2D> inp);
    shared_ptr<geometry::CNPoint2D> mapOutOfOwnPenalty(shared_ptr<geometry::CNPoint2D> inp, shared_ptr<geometry::CNPoint2D> alongVec);
    shared_ptr<geometry::CNPoint2D> mapOutOfOppPenalty(shared_ptr<geometry::CNPoint2D> inp);
    shared_ptr<geometry::CNPoint2D> mapOutOfOppPenalty(shared_ptr<geometry::CNPoint2D> inp, shared_ptr<geometry::CNPoint2D> alongVec);

    shared_ptr<geometry::CNPoint2D> mapOutOfOwnGoalArea(shared_ptr<geometry::CNPoint2D> inp);
    shared_ptr<geometry::CNPoint2D> mapOutOfOppGoalArea(shared_ptr<geometry::CNPoint2D> inp);

    shared_ptr<geometry::CNPoint2D> mapInsideOwnPenaltyArea(shared_ptr<geometry::CNPoint2D> inp, double tolerance);
    shared_ptr<geometry::CNPoint2D> mapInsideOwnPenaltyArea(shared_ptr<geometry::CNPoint2D> inp);

    shared_ptr<geometry::CNPoint2D> mapInsideField(shared_ptr<geometry::CNPoint2D> inp);
    shared_ptr<geometry::CNPoint2D> mapInsideField(shared_ptr<geometry::CNPoint2D> inp, double tolerance);
    shared_ptr<geometry::CNPoint2D> mapInsideField(shared_ptr<geometry::CNPoint2D> inp, shared_ptr<geometry::CNPoint2D> alongVec);

    shared_ptr<geometry::CNPoint2D> keepOutOfOwnPenalty(shared_ptr<geometry::CNPoint2D> from, shared_ptr<geometry::CNPoint2D> to);
    double distanceToLine(shared_ptr<geometry::CNPoint2D> from, double angle);
    double distanceToLine(shared_ptr<geometry::CNPoint2D> from, double angle, double extendFieldLines);

    //  --------- POINTS ---------

    shared_ptr<geometry::CNPoint2D> posCenterMarker(); // see no. 0

    // Corners
    shared_ptr<geometry::CNPoint2D> posLeftOwnCorner();  // see no. 1
    shared_ptr<geometry::CNPoint2D> posRightOwnCorner(); // see no. 2
    shared_ptr<geometry::CNPoint2D> posLeftOppCorner();  // see no. 3
    shared_ptr<geometry::CNPoint2D> posRightOppCorner(); // see no. 4

    // Goalposts
    shared_ptr<geometry::CNPoint2D> posLeftOwnGoalPost();  // see no. 5
    shared_ptr<geometry::CNPoint2D> posRightOwnGoalPost(); // see no. 6
    shared_ptr<geometry::CNPoint2D> posLeftOppGoalPost();  // see no. 7
    shared_ptr<geometry::CNPoint2D> posRightOppGoalPost(); // see no. 8
    // Field Points
    shared_ptr<geometry::CNPoint2D> posOwnPenaltyMarker();      // see no. 9
    shared_ptr<geometry::CNPoint2D> posRightOwnRestartMarker(); // see no. 10
    shared_ptr<geometry::CNPoint2D> posLeftOwnRestartMarker();  // see no. 11
    shared_ptr<geometry::CNPoint2D> posOppPenaltyMarker();      // see no. 12
    shared_ptr<geometry::CNPoint2D> posRightOppRestartMarker(); // see no. 13
    shared_ptr<geometry::CNPoint2D> posLeftOppRestartMarker();  // see no. 14
    shared_ptr<geometry::CNPoint2D> posOppGoalMid();            // see no. 15
    shared_ptr<geometry::CNPoint2D> posOwnGoalMid();            // see no. 16
    shared_ptr<geometry::CNPoint2D> posLROppHalf();             // see no. 17
    shared_ptr<geometry::CNPoint2D> posULOwnHalf();             // see no. 18
    shared_ptr<geometry::CNPoint2D> posLROwnPenaltyArea();      // see no. 19
    shared_ptr<geometry::CNPoint2D> posULOwnPenaltyArea();      // see no. 20
    shared_ptr<geometry::CNPoint2D> posLROppPenaltyArea();      // see no. 21
    shared_ptr<geometry::CNPoint2D> posULOppPenaltyArea();      // see no. 22
    shared_ptr<geometry::CNPoint2D> posLROwnGoalArea();         // see no. 23
    shared_ptr<geometry::CNPoint2D> posULOwnGoalArea();         // see no. 24
    shared_ptr<geometry::CNPoint2D> posLROppGoalArea();         // see no. 25
    shared_ptr<geometry::CNPoint2D> posULOppGoalArea();         // see no. 26
    shared_ptr<geometry::CNPoint2D> posLeftRestartMarker();     // see no. 27
    shared_ptr<geometry::CNPoint2D> posRightRestartMarker();    // see no. 28
    shared_ptr<geometry::CNPoint2D> posLRSurrounding();         // see no. 29
    shared_ptr<geometry::CNPoint2D> posULSurrounding();         // see no. 30
    bool cornerCircleExists();
    double getCornerCircleRadius();
    double getFieldLength();
    double getFieldWidth();
    double getGoalAreaLength();
    double getGoalAreaWidth();
    bool isGoalInnerAreaExists();
    double getGoalWidth();
    double getLineWidth();
    double getMaxDistance();
    double getMaxDistanceSqr();
    double getMiddleCircleRadius();
    double getPenaltyAreaLength();
    double getPenaltyAreaMappingTolerance();
    double getPenaltyAreaWidth();
    double getPenaltySpot();
    double getSurrounding();
    string getCurrentField();

  private:
    supplementary::SystemConfig *sc;
    MSLWorldModel *wm;

    double FieldLength;
    double FieldWidth;
    double PenaltyAreaWidth;
    double PenaltyAreaLength;
    bool GoalInnerAreaExists;
    double GoalAreaLength;
    double GoalAreaWidth;
    bool CornerCircleExists;
    double CornerCircleRadius;
    double MiddleCircleRadius;
    double LineWidth;
    double GoalWidth;
    double PenaltySpot;
    double Surrounding;
    double PenaltyAreaMappingTolerance;
    double MaxDistance;
    double MaxDistanceSqr;
    string CurrentField;
    shared_ptr<geometry::CNPoint2D> mapInsideArea(shared_ptr<geometry::CNPoint2D> inp, double xline, double yline);
    shared_ptr<geometry::CNPoint2D> mapOutsideArea(shared_ptr<geometry::CNPoint2D> inp, double xline, double yline);

  protected:
    double projectVectorOntoX(shared_ptr<geometry::CNPoint2D> origin, shared_ptr<geometry::CNPoint2D> dir, double x);
    double projectVectorOntoY(shared_ptr<geometry::CNPoint2D> origin, shared_ptr<geometry::CNPoint2D> dir, double y);
};
}

#endif
