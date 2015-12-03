/*
 * MSLConstraintBuilder.cpp
 *
 *  Created on: Sep 2, 2014
 *      Author: psp
 */

#include "MSLConstraintBuilder.h"

namespace msl
{
	double MSLConstraintBuilder::AREA_TOL = 100.0;
	double MSLConstraintBuilder::ON_LINE_TOL = 50.0;
	double MSLConstraintBuilder::BLOCK_PASS_WIDTH_TOL = 100.0;
	double MSLConstraintBuilder::BLOCK_MIN_RADIUS = 600.0;
	double MSLConstraintBuilder::MAX_GOAL_DEFEND_DIST = 3000.0;
	double MSLConstraintBuilder::MIN_CORRIDOR_WIDTH = 700.0;
	double MSLConstraintBuilder::MIN_POSITION_DIST = 650.0;

	// INTERN
	msl::MSLFootballField* MSLConstraintBuilder::field = msl::MSLFootballField::getInstance();

	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownRightSurCornerP = field->posLRSurrounding();
	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppLeftSurCornerP = field->posULSurrounding();
	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownRightCornerP = field->posRightOwnCorner();
	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppLeftCornerP = field->posLeftOppCorner();
	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppLRHalfP = field->posLROppHalf();
	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownULHalfP = field->posULOwnHalf();
	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppLRPenaltyAreaP = field->posLROppPenaltyArea();
	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppULPenaltyAreaP = field->posULOppPenaltyArea();
	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownLRPenaltyAreaP = field->posLROwnPenaltyArea();
	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownULPenaltyAreaP = field->posULOwnPenaltyArea();
	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownLRGoalAreaP = field->posLROwnGoalArea();
	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownULGoalAreaP = field->posULOwnGoalArea();
	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppLRGoalAreaP = field->posLROppGoalArea();
	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppULGoalAreaP = field->posULOppGoalArea();
	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownGoalMidP = field->posOwnGoalMid();
	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppGoalMidP = field->posOppGoalMid();
	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::centreMarkP = field->posCenterMarker();

//		shared_ptr<TVec> MSLConstraintBuilder::ownRightSurCornerT;
//		shared_ptr<TVec> MSLConstraintBuilder::oppLeftSurCornerT;
//		shared_ptr<TVec> MSLConstraintBuilder::ownRightCornerT;
//		shared_ptr<TVec> MSLConstraintBuilder::oppLeftCornerT;
//		shared_ptr<TVec> MSLConstraintBuilder::oppLRHalfT;
//		shared_ptr<TVec> MSLConstraintBuilder::ownULHalfT;
	shared_ptr<TVec> MSLConstraintBuilder::oppLRPenaltyAreaT = make_shared<TVec>(initializer_list<double> {
			oppLRPenaltyAreaP->x, oppLRPenaltyAreaP->y});
	shared_ptr<TVec> MSLConstraintBuilder::oppULPenaltyAreaT = make_shared<TVec>(initializer_list<double> {
			oppULPenaltyAreaP->x, oppULPenaltyAreaP->y});
	shared_ptr<TVec> MSLConstraintBuilder::ownLRPenaltyAreaT = make_shared<TVec>(initializer_list<double> {
			ownLRPenaltyAreaP->x, ownLRPenaltyAreaP->y});
	shared_ptr<TVec> MSLConstraintBuilder::ownULPenaltyAreaT = make_shared<TVec>(initializer_list<double> {
			ownULPenaltyAreaP->x, ownULPenaltyAreaP->x});
//		shared_ptr<TVec> MSLConstraintBuilder::ownLRGoalAreaT;
//		shared_ptr<TVec> MSLConstraintBuilder::ownULGoalAreaT;
//		shared_ptr<TVec> MSLConstraintBuilder::oppLRGoalAreaT;
//		shared_ptr<TVec> MSLConstraintBuilder::oppULGoalAreaT;
//		shared_ptr<TVec> MSLConstraintBuilder::ownGoalMidT;
//		shared_ptr<TVec> MSLConstraintBuilder::oppGoalMidT;
//		shared_ptr<TVec> MSLConstraintBuilder::centreMarkT;


	shared_ptr<Term> MSLConstraintBuilder::spreadUtil(vector<shared_ptr<TVec>> points) {
		shared_ptr<Term> c;
		return c;
	}
	shared_ptr<Term> MSLConstraintBuilder::spread(double mindist, vector<shared_ptr<TVec>> points) {
		if (points.size() == 1)
			return autodiff::LTConstraint::TRUE;
		shared_ptr<Term> spreadConstraint = autodiff::LTConstraint::TRUE;
		//TVec[] curPoints = points;
		for (int i = 0; i < points.size() - 1; i++) {
			for(int j=i+1; j<points.size();j++) {
				//TODO
//				spreadConstraint = spreadConstraint & outsideSphere(points[i],minDist,points[j]);
			}
		}
		return spreadConstraint;
	}
	shared_ptr<Term> MSLConstraintBuilder::outsideRectangle(shared_ptr<TVec> lowerRightCorner,
															shared_ptr<TVec> upperLeftCorner,
															vector<shared_ptr<TVec>> points)
	{
		shared_ptr<Term> c = !TermBuilder::boundedRectangle(points[0], lowerRightCorner, upperLeftCorner,
															Term::getConstraintSteepness());

		for (int i = 1; i < points.size(); ++i)
		{
			c = c
					& !TermBuilder::boundedRectangle(points[i], lowerRightCorner, upperLeftCorner,
														Term::getConstraintSteepness());
		}
		return c;
	}

	shared_ptr<Term> MSLConstraintBuilder::insideRectangle(shared_ptr<TVec> lowerRightCorner,
															shared_ptr<TVec> upperLeftCorner,
															vector<shared_ptr<TVec>> points)
	{
		shared_ptr<Term> c = TermBuilder::boundedRectangle(points[0], lowerRightCorner, upperLeftCorner,
															Term::getConstraintSteepness());

		for (int i = 1; i < points.size(); ++i)
		{
			c = c
					& TermBuilder::boundedRectangle(points[i], lowerRightCorner, upperLeftCorner,
														Term::getConstraintSteepness());
		}
		return c;
	}

	shared_ptr<Term> MSLConstraintBuilder::applyRules(int specialIdx, vector<shared_ptr<TVec>> fieldPlayers) {
		msl::MSLWorldModel* wm = msl::MSLWorldModel::get();
		shared_ptr<Term> c;
		shared_ptr<TVec> ballT;
		shared_ptr<geometry::CNPoint2D> ball = wm->ball.getEgoBallPosition()->egoToAllo(*wm->rawSensorData.getOwnPositionVision());
		if(ball != nullptr) {
			ballT = make_shared<TVec>(initializer_list<double> {ball->x, ball->x});
		}
		switch(wm->game.getSituation()) {
			case Situation::Start:
			case Situation::Restart:
			case Situation::Ready:
			case Situation::FirstHalf:
			case Situation::SecondHalf:
				c = c & commonRules(fieldPlayers);
				break;
			case Situation::OwnCorner:
			case Situation::OwnFreekick:
			case Situation::OwnGoalkick:
			case Situation::OwnThrowin:
				c = c & ownStdRules (ballT, specialIdx, fieldPlayers);
				break;
			case Situation::OwnKickoff:
				c = c & ownKickOffRules(ballT, specialIdx, fieldPlayers);
				break;
			case Situation::OwnPenalty:
				c = c & ownPenaltyRules (ballT, specialIdx, fieldPlayers);
				break;
			case Situation::OppCorner:
			case Situation::OppFreekick:
			case Situation::OppGoalkick:
			case Situation::OppThrowin:
				c = c & oppStdRules (ballT, fieldPlayers);
				break;
			case Situation::OppKickoff:
				c = c & oppKickOffRules (ballT, fieldPlayers);
				break;
			case Situation::OppPenalty:
				c = c & oppPenaltyRules (fieldPlayers);
				break;
			case Situation::DropBall:
				c = c & dropBallRules (ballT, fieldPlayers);
				break;
			case Situation::Stop:
				// no constraints
				c = autodiff::LTConstraint::TRUE;
				break;
			default:
				break;
		}
		return c;
	}
	shared_ptr<Term> MSLConstraintBuilder::commonRules(vector<shared_ptr<TVec>> fieldPlayers) {
		shared_ptr<Term> c;
		return c;
	}
	shared_ptr<Term> MSLConstraintBuilder::dropBallRules(shared_ptr<TVec> ballT, vector<shared_ptr<TVec>> fieldPlayers) {
		shared_ptr<Term> c;
		return c;
	}
	shared_ptr<Term> MSLConstraintBuilder::ownPenaltyRules(shared_ptr<TVec> ballT, int executerIdx, vector<shared_ptr<TVec>> fieldPlayers) {
		shared_ptr<Term> c;
		return c;
	}
	shared_ptr<Term> MSLConstraintBuilder::oppPenaltyRules(vector<shared_ptr<TVec>> fieldPlayers) {
		shared_ptr<Term> c;
		return c;
	}
	shared_ptr<Term> MSLConstraintBuilder::ownKickOffRules(shared_ptr<TVec> ballT, int executerIdx, vector<shared_ptr<TVec>> fieldPlayers) {
		shared_ptr<Term> c;
		return c;
	}
	shared_ptr<Term> MSLConstraintBuilder::oppKickOffRules(shared_ptr<TVec> ballT, vector<shared_ptr<TVec>> fieldPlayers) {
		shared_ptr<Term> c;
		return c;
	}
	shared_ptr<Term> MSLConstraintBuilder::ownStdRules(shared_ptr<TVec> ballT, int executerIdx, vector<shared_ptr<TVec>> fieldPlayers) {
		shared_ptr<Term> c;
		return c;
	}
	shared_ptr<Term> MSLConstraintBuilder::oppStdRules(shared_ptr<TVec> ballT, vector<shared_ptr<TVec>> fieldPlayers) {
		shared_ptr<Term> c;
		return c;
	}
	shared_ptr<Term> MSLConstraintBuilder::ownPenaltyAreaDistanceExceptionRule(shared_ptr<TVec> ballT, vector<shared_ptr<TVec>> fieldPlayers) {
		shared_ptr<Term> c;
		return c;
	}
	shared_ptr<Term> MSLConstraintBuilder::ownPenaltyAreaRule(vector<shared_ptr<TVec>> fieldPlayers) {
		shared_ptr<Term> c;
		return c;
	}
	shared_ptr<Term> MSLConstraintBuilder::oppPenaltyAreaRule(vector<shared_ptr<TVec>> fieldPlayers) {
		shared_ptr<Term> c;
		return c;
	}

	shared_ptr<Term> MSLConstraintBuilder::outsideArea(Areas area, shared_ptr<TVec> point)
	{
		vector<shared_ptr<TVec>> points;
		points.push_back(point);
		return outsideArea(area, points);
	}

	shared_ptr<Term> MSLConstraintBuilder::outsideArea(Areas area, vector<shared_ptr<TVec>> points)
	{
		shared_ptr<geometry::CNPoint2D> lowerRightCornerP;
		shared_ptr<geometry::CNPoint2D> upperLeftCornerP;
		resolveArea(area, &lowerRightCornerP, &upperLeftCornerP);
		shared_ptr<TVec> lowerRightCorner = make_shared<TVec>(
				initializer_list<double> {lowerRightCornerP->x + AREA_TOL, lowerRightCornerP->y + AREA_TOL});
		shared_ptr<TVec> upperLeftCorner = make_shared<TVec>(
				initializer_list<double> {upperLeftCornerP->x - AREA_TOL, upperLeftCornerP->y - AREA_TOL});
		return outsideRectangle(lowerRightCorner, upperLeftCorner, points);
	}

	shared_ptr<Term> MSLConstraintBuilder::insideArea(Areas area, shared_ptr<TVec> point)
	{

		vector<shared_ptr<TVec>> points;
		points.push_back(point);
		return insideArea(area, points);
	}

	shared_ptr<Term> MSLConstraintBuilder::insideArea(Areas area, vector<shared_ptr<TVec>> points)
	{
		shared_ptr<geometry::CNPoint2D> lowerRightCornerP;
		shared_ptr<geometry::CNPoint2D> upperLeftCornerP;
		resolveArea(area, &lowerRightCornerP, &upperLeftCornerP);
		shared_ptr<TVec> lowerRightCorner = make_shared<TVec>(
				initializer_list<double> {lowerRightCornerP->x - AREA_TOL, lowerRightCornerP->y - AREA_TOL});
		shared_ptr<TVec> upperLeftCorner = make_shared<TVec>(
				initializer_list<double> {upperLeftCornerP->x + AREA_TOL, upperLeftCornerP->y + AREA_TOL});
		return insideRectangle(lowerRightCorner, upperLeftCorner, points);
	}

	void MSLConstraintBuilder::resolveArea(Areas area, shared_ptr<geometry::CNPoint2D> *lowerRightCorner,
											shared_ptr<geometry::CNPoint2D> *upperLeftCorner)
	{
		switch (area)
		{
			case Areas::Surrounding:
				*lowerRightCorner = ownRightSurCornerP;
				*upperLeftCorner = oppLeftCornerP;
				break;
			case Areas::Field:
				*lowerRightCorner = ownRightCornerP;
				*upperLeftCorner = oppLeftCornerP;
				break;
			case Areas::OppHalf:
				*lowerRightCorner = oppLRHalfP;
				*upperLeftCorner = oppLeftCornerP;
				break;
			case Areas::OwnHalf:
				*lowerRightCorner = ownRightCornerP;
				*upperLeftCorner = ownULHalfP;
				break;
			case Areas::OppPenaltyArea:
				*lowerRightCorner = oppLRPenaltyAreaP;
				*upperLeftCorner = oppULPenaltyAreaP;
				break;
			case Areas::OwnPenaltyArea:
				*lowerRightCorner = ownLRPenaltyAreaP;
				*upperLeftCorner = ownULPenaltyAreaP;
				break;
			case Areas::OwnGoalArea:
				*lowerRightCorner = ownLRGoalAreaP;
				*upperLeftCorner = ownULGoalAreaP;
				break;
			case Areas::OppGoalArea:
				*lowerRightCorner = oppLRGoalAreaP;
				*upperLeftCorner = oppULGoalAreaP;
				break;
			default:
				throw "Unknown Area!";
		}
	}
} /* namespace msl */
