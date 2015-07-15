/*
 * $Id: FootballField.h 1531 2006-08-01 21:36:57Z phbaer $
 *
 *
 * Copyright 2005,2006 Carpe Noctem, Distributed Systems Group,
 * University of Kassel. All right reserved.
 *
 * The code is derived from the software contributed to Carpe Noctem by
 * the Carpe Noctem Team.
 *
 * The code is licensed under the Carpe Noctem Userfriendly BSD-Based
 * License (CNUBBL). Redistribution and use in source and binary forms,
 * with or without modification, are permitted provided that the
 * conditions of the CNUBBL are met.
 *
 * You should have received a copy of the CNUBBL along with this
 * software. The license is also available on our website:
 * http://carpenoctem.das-lab.net/license.txt
 *
 *
 * <description>
 */
#ifndef MSLFootballField_H
#define MSLFootballField_H

#include <SystemConfig.h>
#include <GeometryCalculator.h>

using namespace supplementary;

//                   T  H  E    E  N  E  M  Y  `  S    S  I  D  E
//
//30---------------------------------------------------------------------------+
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
// OppGoal angle 0
// truning left positive angle
// turning right negative angle
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
	class MSLFootballField
	{

	public:

		static MSLFootballField * getInstance();

		static double FieldLength;
		static double FieldWidth;
		static double GoalAreaWidth;
		static double GoalAreaLength;
		static double GoalInnerAreaWidth;
		static double GoalInnerAreaLength;
		static double CornerCircleRadius;
		static double MiddleCircleRadius;
		static double LineWidth;
		static double GoalWidth;
		static bool GoalInnerAreaExists;
		static bool CornerCircleExists;
		static double PenaltySpot;
		static double Surrounding;

		static bool isInsideField(shared_ptr<geometry::CNPoint2D> point, double tolerance);
		static bool isInsideOwnPenalty(shared_ptr<geometry::CNPoint2D> p, double tolerance);
		static bool isInsideEnemyPenalty(shared_ptr<geometry::CNPoint2D> p, double tolerance);
		static bool isInsidePenalty (shared_ptr<geometry::CNPoint2D> p, double tolerance);
		static shared_ptr<geometry::CNPoint2D> posCenterMarker(); // see no. 0

		// Corners
		static shared_ptr<geometry::CNPoint2D> posLeftOwnCorner(); // see no. 1
		static shared_ptr<geometry::CNPoint2D> posRightOwnCorner(); // see no. 2
		static shared_ptr<geometry::CNPoint2D> posLeftOppCorner(); // see no. 3
		static shared_ptr<geometry::CNPoint2D> posRightOppCorner(); // see no. 4

		// Goalposts
		static shared_ptr<geometry::CNPoint2D> posLeftOwnGoalPost();  // see no. 5
		static shared_ptr<geometry::CNPoint2D> posRightOwnGoalPost();  // see no. 6
		static shared_ptr<geometry::CNPoint2D> posLeftOppGoalPost(); // see no. 7
		static shared_ptr<geometry::CNPoint2D> posRightOppGoalPost(); // see no. 8
		// Field Points
		static shared_ptr<geometry::CNPoint2D> posOwnPenaltyMarker(); // see no. 9
		static shared_ptr<geometry::CNPoint2D> posRightOwnRestartMarker(); // see no. 10
		static shared_ptr<geometry::CNPoint2D> posLeftOwnRestartMarker(); // see no. 11
		static shared_ptr<geometry::CNPoint2D> posOppPenaltyMarker(); // see no. 12
		static shared_ptr<geometry::CNPoint2D> posRightOppRestartMarker(); // see no. 13
		static shared_ptr<geometry::CNPoint2D> posLeftOppRestartMarker(); // see no. 14
		static shared_ptr<geometry::CNPoint2D> posOppGoalMid(); // see no. 15
		static shared_ptr<geometry::CNPoint2D> posOwnGoalMid(); // see no. 16
		static shared_ptr<geometry::CNPoint2D> posLROppHalf(); // see no. 17
		static shared_ptr<geometry::CNPoint2D> posULOwnHalf(); // see no. 18
		static shared_ptr<geometry::CNPoint2D> osLROwnPenaltyArea(); // see no. 19
		static shared_ptr<geometry::CNPoint2D> posULOwnPenaltyArea(); // see no. 20
		static shared_ptr<geometry::CNPoint2D> posLROppPenaltyArea(); // see no. 21
		static shared_ptr<geometry::CNPoint2D> posULOppPenaltyArea(); // see no. 22
		static shared_ptr<geometry::CNPoint2D> posLROwnGoalArea(); // see no. 23
		static shared_ptr<geometry::CNPoint2D> posULOwnGoalArea(); // see no. 24
		static shared_ptr<geometry::CNPoint2D> posLROppGoalArea(); // see no. 25
		static shared_ptr<geometry::CNPoint2D> posULOppGoalArea();// see no. 26
		static shared_ptr<geometry::CNPoint2D> posLeftRestartMarker(); // see no. 27
		static shared_ptr<geometry::CNPoint2D> posRightRestartMarker(); // see no. 28
		static shared_ptr<geometry::CNPoint2D> posLRSurrounding();// see no. 29
		static shared_ptr<geometry::CNPoint2D> posULSurrounding(); // see no. 30

	private:

		SystemConfig* sc;

		static MSLFootballField * instance;
		MSLFootballField();
		~MSLFootballField();

	};

}

#endif

