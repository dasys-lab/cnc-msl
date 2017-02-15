#pragma once

#include <cnc_geometry/Calculator.h>
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

    bool isInsideField(geometry::CNPointAllo point, double tolerance = 0);
    bool isInsideField(double x, double y, double tolerance = 0);

    bool isInsidePenalty(const geometry::CNPointAllo p, double tolerance) const;
    bool isInsideOwnPenalty(const geometry::CNPointAllo p, double tolerance) const;
    bool isInsideOppPenalty(const geometry::CNPointAllo p, double tolerance) const;

    bool isInsideGoalArea(geometry::CNPointAllo p, double tolerance);
    bool isInsideOwnGoalArea(geometry::CNPointAllo p, double tolerance);
    bool isInsideOppGoalArea(geometry::CNPointAllo p, double tolerance);

    geometry::CNPointAllo mapOutOfPenalty(geometry::CNPointAllo inp);
    geometry::CNPointAllo mapOutOfOwnPenalty(geometry::CNPointAllo inp);
    geometry::CNPointAllo mapOutOfOwnPenalty(geometry::CNPointAllo inp, geometry::CNPointAllo alongVec);
    geometry::CNPointAllo mapOutOfOppPenalty(geometry::CNPointAllo inp);
    geometry::CNPointAllo mapOutOfOppPenalty(geometry::CNPointAllo inp, geometry::CNPointAllo alongVec);

    geometry::CNPointAllo mapOutOfOwnGoalArea(geometry::CNPointAllo inp);
    geometry::CNPointAllo mapOutOfOppGoalArea(geometry::CNPointAllo inp);

    geometry::CNPointAllo mapInsideOwnPenaltyArea(geometry::CNPointAllo inp, double tolerance);
    geometry::CNPointAllo mapInsideOwnPenaltyArea(geometry::CNPointAllo inp);

    geometry::CNPointAllo mapInsideField(geometry::CNPointAllo inp);
    geometry::CNPointAllo mapInsideField(geometry::CNPointAllo inp, double tolerance);
    geometry::CNPointAllo mapInsideField(geometry::CNPointAllo inp, geometry::CNPointAllo alongVec);

    geometry::CNPointAllo keepOutOfOwnPenalty(geometry::CNPointAllo from, geometry::CNPointAllo to);
    double distanceToLine(geometry::CNPointAllo from, double angle);
    double distanceToLine(geometry::CNPointAllo from, double angle, double extendFieldLines);

    //  --------- POINTS ---------

    geometry::CNPointAllo posCenterMarker(); // see no. 0

    // Corners
    geometry::CNPointAllo posLeftOwnCorner();  // see no. 1
    geometry::CNPointAllo posRightOwnCorner(); // see no. 2
    geometry::CNPointAllo posLeftOppCorner();  // see no. 3
    geometry::CNPointAllo posRightOppCorner(); // see no. 4

    // Goalposts
    geometry::CNPointAllo posLeftOwnGoalPost();  // see no. 5
    geometry::CNPointAllo posRightOwnGoalPost(); // see no. 6
    geometry::CNPointAllo posLeftOppGoalPost();  // see no. 7
    geometry::CNPointAllo posRightOppGoalPost(); // see no. 8
    // Field Points
    geometry::CNPointAllo posOwnPenaltyMarker();      // see no. 9
    geometry::CNPointAllo posRightOwnRestartMarker(); // see no. 10
    geometry::CNPointAllo posLeftOwnRestartMarker();  // see no. 11
    geometry::CNPointAllo posOppPenaltyMarker();      // see no. 12
    geometry::CNPointAllo posRightOppRestartMarker(); // see no. 13
    geometry::CNPointAllo posLeftOppRestartMarker();  // see no. 14
    geometry::CNPointAllo posOppGoalMid();            // see no. 15
    geometry::CNPointAllo posOwnGoalMid();            // see no. 16
    geometry::CNPointAllo posLROppHalf();             // see no. 17
    geometry::CNPointAllo posULOwnHalf();             // see no. 18
    geometry::CNPointAllo posLROwnPenaltyArea();      // see no. 19
    geometry::CNPointAllo posULOwnPenaltyArea();      // see no. 20
    geometry::CNPointAllo posLROppPenaltyArea();      // see no. 21
    geometry::CNPointAllo posULOppPenaltyArea();      // see no. 22
    geometry::CNPointAllo posLROwnGoalArea();         // see no. 23
    geometry::CNPointAllo posULOwnGoalArea();         // see no. 24
    geometry::CNPointAllo posLROppGoalArea();         // see no. 25
    geometry::CNPointAllo posULOppGoalArea();         // see no. 26
    geometry::CNPointAllo posLeftRestartMarker();     // see no. 27
    geometry::CNPointAllo posRightRestartMarker();    // see no. 28
    geometry::CNPointAllo posLRSurrounding();         // see no. 29
    geometry::CNPointAllo posULSurrounding();         // see no. 30
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
    geometry::CNPointAllo mapInsideArea(geometry::CNPointAllo inp, double xline, double yline);
    geometry::CNPointAllo mapOutsideArea(geometry::CNPointAllo inp, double xline, double yline);

  protected:
    double projectVectorOntoX(geometry::CNPointAllo origin, geometry::CNPointAllo dir, double x);
    double projectVectorOntoY(geometry::CNPointAllo origin, geometry::CNPointAllo dir, double y);
};
}
