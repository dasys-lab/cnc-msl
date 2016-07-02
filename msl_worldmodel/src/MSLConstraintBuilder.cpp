/*
 * MSLConstraintBuilder.cpp
 *
 *  Created on: Sep 2, 2014
 *      Author: psp
 */

#include "MSLConstraintBuilder.h"
#include "Rules.h"
#include "MSLWorldModel.h"
#include "RawSensorData.h"
#include "obstaclehandler/Obstacles.h"
#include "Robots.h"
#include "Ball.h"
#include "Game.h"

namespace msl
{
	double MSLConstraintBuilder::AREA_TOL = 180.0;
	double MSLConstraintBuilder::ON_LINE_TOL = 50.0;
	double MSLConstraintBuilder::BLOCK_PASS_WIDTH_TOL = 100.0;
	double MSLConstraintBuilder::BLOCK_MIN_RADIUS = 600.0;
	double MSLConstraintBuilder::MAX_GOAL_DEFEND_DIST = 3000.0;
	double MSLConstraintBuilder::MIN_CORRIDOR_WIDTH = 700.0;
	double MSLConstraintBuilder::MIN_POSITION_DIST = 650.0;

	supplementary::SystemConfig* MSLConstraintBuilder::sc = supplementary::SystemConfig::getInstance();

	Rules* MSLConstraintBuilder::rules = Rules::getInstance();

	// INTERN
	//TODO check

//	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownRightSurCornerP = wm->field.posLRSurrounding();
//	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppLeftSurCornerP = wm->field.posULSurrounding();
//	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownRightCornerP = wm->field.posRightOwnCorner();
//	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppLeftCornerP = wm->field.posLeftOppCorner();
//	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppLRHalfP = wm->field.posLROppHalf();
//	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownULHalfP = wm->field.posULOwnHalf();
//	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppLRPenaltyAreaP = wm->field.posLROppPenaltyArea();
//	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppULPenaltyAreaP = wm->field.posULOppPenaltyArea();
//	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownLRPenaltyAreaP = wm->field.posLROwnPenaltyArea();
//	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownULPenaltyAreaP = wm->field.posULOwnPenaltyArea();
//	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownLRGoalAreaP = wm->field.posLROwnGoalArea();
//	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownULGoalAreaP = wm->field.posULOwnGoalArea();
//	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppLRGoalAreaP = wm->field.posLROppGoalArea();
//	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppULGoalAreaP = wm->field.posULOppGoalArea();
//	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::ownGoalMidP = wm->field.posOwnGoalMid();
//	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppGoalMidP = wm->field.posOppGoalMid();
//	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::centreMarkP = wm->field.posCenterMarker();

//	shared_ptr<TVec> MSLConstraintBuilder::ownRightSurCornerT = make_shared<TVec>(initializer_list<double> {ownRightSurCornerP->x, ownRightSurCornerP->y});
//	shared_ptr<TVec> MSLConstraintBuilder::oppLeftSurCornerT = make_shared<TVec>(initializer_list<double> {oppLeftSurCornerP->x, oppLeftSurCornerP->y});
//	shared_ptr<TVec> MSLConstraintBuilder::ownRightCornerT = make_shared<TVec>(initializer_list<double> {ownRightCornerP->x, ownRightCornerP->y});
//	shared_ptr<TVec> MSLConstraintBuilder::oppLeftCornerT = make_shared<TVec>(initializer_list<double> {oppLeftCornerP->x, oppLeftCornerP->y});
//	shared_ptr<TVec> MSLConstraintBuilder::oppLRHalfT = make_shared<TVec>(initializer_list<double> {oppLRHalfP->x, oppLRHalfP->y});
//	shared_ptr<TVec> MSLConstraintBuilder::ownULHalfT = make_shared<TVec>(initializer_list<double> {ownULHalfP->x, ownULHalfP->y});
//	shared_ptr<TVec> MSLConstraintBuilder::oppLRPenaltyAreaT = make_shared<TVec>(initializer_list<double> {oppLRPenaltyAreaP->x, oppLRPenaltyAreaP->y});
//	shared_ptr<TVec> MSLConstraintBuilder::oppULPenaltyAreaT = make_shared<TVec>(initializer_list<double> {oppULPenaltyAreaP->x, oppULPenaltyAreaP->y});
//	shared_ptr<TVec> MSLConstraintBuilder::ownLRPenaltyAreaT = make_shared<TVec>(initializer_list<double> {ownLRPenaltyAreaP->x, ownLRPenaltyAreaP->y});
//	shared_ptr<TVec> MSLConstraintBuilder::ownULPenaltyAreaT = make_shared<TVec>(initializer_list<double> {ownULPenaltyAreaP->x, ownULPenaltyAreaP->x});
//	shared_ptr<TVec> MSLConstraintBuilder::ownLRGoalAreaT = make_shared<TVec>(initializer_list<double> {ownLRGoalAreaP->x, ownLRGoalAreaP->y});
//	shared_ptr<TVec> MSLConstraintBuilder::ownULGoalAreaT = make_shared<TVec>(initializer_list<double> {ownULGoalAreaP->x, ownULGoalAreaP->y});
//	shared_ptr<TVec> MSLConstraintBuilder::oppLRGoalAreaT = make_shared<TVec>(initializer_list<double> {oppLRGoalAreaP->x, oppLRGoalAreaP->y});
//	shared_ptr<TVec> MSLConstraintBuilder::oppULGoalAreaT = make_shared<TVec>(initializer_list<double> {oppULGoalAreaP->x, oppULGoalAreaP->y});
//	shared_ptr<TVec> MSLConstraintBuilder::ownGoalMidT = make_shared<TVec>(initializer_list<double> {ownGoalMidP->x, ownGoalMidP->y});
//	shared_ptr<TVec> MSLConstraintBuilder::oppGoalMidT = make_shared<TVec>(initializer_list<double> {oppGoalMidP->x, oppGoalMidP->y});
	shared_ptr<TVec> MSLConstraintBuilder::centreMarkT = make_shared<TVec>(initializer_list<double> {0.0, 0.0});

	shared_ptr<Term> MSLConstraintBuilder::spreadUtil(vector<shared_ptr<TVec>>& points)
	{
		if (points.size() == 1)
			return autodiff::TermBuilder::constant(0);
		shared_ptr<Term> spreadUtility = autodiff::TermBuilder::constant(0);
		vector<shared_ptr<TVec>> curPoints = points;
		MSLWorldModel* wm = MSLWorldModel::get();
		double maxFieldDist = std::sqrt(
				wm->field->getFieldLength() * wm->field->getFieldLength()
						+ wm->field->getFieldWidth() * wm->field->getFieldWidth());

		for (int i = 0; i < points.size(); i++)
		{
			std::copy_if(std::begin(curPoints), std::end(curPoints), std::back_inserter(curPoints),
							[&](shared_ptr<TVec> v)
							{	return v != curPoints[0];});
			for (int j = 0; j < curPoints.size(); j++)
			{
				spreadUtility = spreadUtility
						+ alica::ConstraintBuilder::distanceSqr(points[i],
																curPoints[j] * (1 / (maxFieldDist * maxFieldDist)));
			}
		}
		return spreadUtility;
	}

	shared_ptr<Term> MSLConstraintBuilder::see(shared_ptr<TVec> point, bool considerownPos, double detectionRadius,
												vector<shared_ptr<TVec>>& players)
	{
		msl::MSLWorldModel* wm = msl::MSLWorldModel::get();
		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData->getOwnPositionVision();

		if (ownPos == nullptr)
			return autodiff::Term::FALSE;

		// all players have to be in sight radius
		shared_ptr<Term> c = insideSphere(point, detectionRadius, players);
		for (int i = 0; i < players.size(); ++i)
		{
			shared_ptr<TVec> p2p = players.at(i) - point;
			shared_ptr<Term> rel = (wm->obstacles->getObstacleRadius()
					/ autodiff::TermBuilder::power(p2p->normSquared(), 0.5));

			// consider opponents
			auto opps = wm->robots->opponents.getOpponentsAlloClustered();

//				List<TrackedOpponent> opps = wm.GetTrackedOpponents();

			if (opps != nullptr && opps->size() > 0)
			{
				for (auto opp : *opps)
				{
					shared_ptr<TVec> oppT = make_shared<TVec>(initializer_list<double> {opp->x, opp->y});
					shared_ptr<TVec> p2opp = oppT - point;

					shared_ptr<TVec> oppDistance = alica::ConstraintBuilder::inCoordsOf(p2opp, p2p);
					c = c
							& alica::ConstraintBuilder::ifThen(
									((oppDistance->getX() > autodiff::TermBuilder::constant(0))
											& (oppDistance->getX() < autodiff::TermBuilder::constant(1))),
									rel
											< ((make_shared<Abs>(oppDistance->getY())
													- ((wm->obstacles->getObstacleRadius() + 50.0)
															/ autodiff::TermBuilder::power(p2p->normSquared(), 0.5)))
													/ oppDistance->getX()));
				}
			}

			// consider teammates
			auto mates = wm->robots->teammates.getTeammatesAlloClustered();
			if (mates != nullptr && mates->size() > 0)
			{
				for (auto mate : *mates)
				{
					shared_ptr<TVec> mateT = make_shared<TVec>(initializer_list<double> {mate->x, mate->y});
					shared_ptr<TVec> p2mate = mateT - point;

					shared_ptr<TVec> mateDistance = alica::ConstraintBuilder::inCoordsOf(p2mate, p2p);
					c = c
							& alica::ConstraintBuilder::ifThen(
									((mateDistance->getX() > autodiff::TermBuilder::constant(0))
											& (mateDistance->getX() < autodiff::TermBuilder::constant(1))),
									rel
											< ((make_shared<Abs>(mateDistance->getY())
													- ((wm->obstacles->getObstacleRadius() + 50.0)
															/ autodiff::TermBuilder::power(p2p->normSquared(), 0.5)))
													/ mateDistance->getX()));
				}
			}

			// consider myself
			if (considerownPos)
			{
				shared_ptr<TVec> ownPosT = make_shared<TVec>(initializer_list<double> {ownPos->x, ownPos->y});
				shared_ptr<TVec> p2ownPos = ownPosT - point;

				shared_ptr<TVec> myPosDistance = alica::ConstraintBuilder::inCoordsOf(p2ownPos, p2p);
				c = c
						& alica::ConstraintBuilder::ifThen(
								((myPosDistance->getX() > autodiff::TermBuilder::constant(0))
										& (myPosDistance->getX() < autodiff::TermBuilder::constant(1))),
								rel
										< ((make_shared<Abs>(myPosDistance->getY())
												- ((wm->obstacles->getObstacleRadius() + 50.0)
														/ autodiff::TermBuilder::power(p2p->normSquared(), 0.5)))
												/ myPosDistance->getX()));
			}
		}
		return c;
	}

	shared_ptr<Term> MSLConstraintBuilder::approachUtil(shared_ptr<TVec> destination, vector<shared_ptr<TVec>>& points)
	{
		shared_ptr<Term> util = autodiff::TermBuilder::constant(0);
		MSLWorldModel* wm = MSLWorldModel::get();
		double maxFieldDist = std::sqrt(
				wm->field->getFieldLength() * wm->field->getFieldLength()
						+ wm->field->getFieldWidth() * wm->field->getFieldWidth());

		for (int i = 0; i < points.size(); i++)
		{
			auto value = (1
					- (alica::ConstraintBuilder::distanceSqr(destination, points[i])) / (maxFieldDist * maxFieldDist));
			util = util + value;
		}
		return util;
	}

	shared_ptr<Term> MSLConstraintBuilder::approachUtil(shared_ptr<TVec> destination, shared_ptr<TVec> point)
	{
		MSLWorldModel* wm = MSLWorldModel::get();
		double maxFieldDist = std::sqrt(
				wm->field->getFieldLength() * wm->field->getFieldLength()
						+ wm->field->getFieldWidth() * wm->field->getFieldWidth());
		return (1 - (alica::ConstraintBuilder::distanceSqr(destination, point)) / (maxFieldDist * maxFieldDist));
	}

	/*
	 * hessennormalform: x * n = d
	 * @param norm: normal vector from line
	 * @param d: d value from hessennormalform
	 * @param points: positions from robots
	 */
	shared_ptr<Term> MSLConstraintBuilder::lineUpUtil(shared_ptr<geometry::CNPoint2D> norm, double d,
														vector<shared_ptr<TVec>>& points)
	{
		MSLWorldModel* wm = MSLWorldModel::get();
		shared_ptr<Term> util = autodiff::TermBuilder::constant(0);
		double maxFieldDist = std::sqrt(
				wm->field->getFieldLength() * wm->field->getFieldLength()
						+ wm->field->getFieldWidth() * wm->field->getFieldWidth());

		for (int i = 0; i < points.size(); i++)
		{
			auto value = ((points[i]->getX() * norm->x + points[i]->getY() * norm->y) - d) / maxFieldDist;
			util = util + (1 - value * value);
		}

		return util;
	}

	shared_ptr<Term> MSLConstraintBuilder::lazyUtil(vector<shared_ptr<TVec>>& robots, vector<shared_ptr<TVec>>& points)
	{
		MSLWorldModel* wm = MSLWorldModel::get();
		shared_ptr<Term> util = autodiff::TermBuilder::constant(0);
		double maxFieldDist = std::sqrt(
				wm->field->getFieldLength() * wm->field->getFieldLength()
						+ wm->field->getFieldWidth() * wm->field->getFieldWidth());

		for (int i = 0; i < points.size(); i++)
		{
			if (robots[i] == nullptr)
				continue;

			auto value = 1
					- (alica::ConstraintBuilder::distanceSqr(points[i], robots[i]) / (maxFieldDist * maxFieldDist));
			util = util + value;
		}
		return util;
	}

	shared_ptr<Term> MSLConstraintBuilder::spread(double minDist, vector<shared_ptr<TVec>>& points)
	{
		if (points.size() == 1)
			return autodiff::LTConstraint::TRUE;
		shared_ptr<Term> c = autodiff::LTConstraint::TRUE;
		for (int i = 0; i < points.size() - 1; i++)
		{
			for (int j = i + 1; j < points.size(); j++)
			{
				c = c & outsideSphere(points[i], minDist, points[j]);
			}
		}
		return c;
	}
	shared_ptr<Term> MSLConstraintBuilder::outsideRectangle(shared_ptr<TVec> lowerRightCorner,
															shared_ptr<TVec> upperLeftCorner,
															vector<shared_ptr<TVec>>& points)
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
	shared_ptr<Term> MSLConstraintBuilder::outsideCorridor(shared_ptr<geometry::CNPoint2D> a,
															shared_ptr<geometry::CNPoint2D> b, double width,
															vector<shared_ptr<TVec>>& points)
	{
		shared_ptr<Term> c = autodiff::LTConstraint::TRUE;

		return c;
	}

	shared_ptr<Term> MSLConstraintBuilder::insideCorridor(shared_ptr<geometry::CNPoint2D> a,
															shared_ptr<geometry::CNPoint2D> b, double width,
															vector<shared_ptr<TVec>>& points)
	{
		shared_ptr<Term> c = autodiff::LTConstraint::TRUE;

		shared_ptr<geometry::CNPoint2D> a2b = b - a;
		shared_ptr<geometry::CNPoint2D> normal = (a2b->rotate(M_PI / 2))->normalize();

		shared_ptr<geometry::CNPoint2D> r1 = a - (normal * width);
		shared_ptr<geometry::CNPoint2D> r2 = b + (normal * width);

		cout << "MSLCB: r1 x,y: " << r1->x << " " << r1->y << endl;
		cout << "MSLCB: r2 x,y: " << r2->x << " " << r2->y << endl;

		shared_ptr<TVec> tvec1 = make_shared<TVec>(initializer_list<double> {r1->x, r1->y});
		shared_ptr<TVec> tvec2 = make_shared<TVec>(initializer_list<double> {r2->x, r2->y});

		for (int i = 0; i < points.size(); i++)
		{
//			cout << "MSLCB: " << TermBuilder::boundedRectangle(points[i], tvec1, tvec2 ,Term::getConstraintSteepness())->toString() << endl;
			c = c & TermBuilder::boundedRectangle(points[i], tvec1, tvec2, Term::getConstraintSteepness());
		}
		return c;
	}

	shared_ptr<Term> MSLConstraintBuilder::insideRectangle(shared_ptr<TVec> lowerRightCorner,
															shared_ptr<TVec> upperLeftCorner,
															vector<shared_ptr<TVec>>& points)
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
	shared_ptr<Term> MSLConstraintBuilder::outsideSphere(shared_ptr<TVec> point, double distance,
															vector<shared_ptr<TVec>>& points)
	{
		if (point != nullptr && points.size() > 0)
		{
			shared_ptr<Term> c = TermBuilder::euclidianDistanceSqr(point, points[0])
					> autodiff::TermBuilder::constant(distance * distance);
			for (int i = 1; i < points.size(); i++)
			{
				c = c
						& TermBuilder::euclidianDistanceSqr(point, points[i])
								> autodiff::TermBuilder::constant(distance * distance);
			}
			return c;
		}
		else
		{
			return nullptr;
		}
	}
	shared_ptr<Term> MSLConstraintBuilder::outsideSphere(shared_ptr<TVec> point, double distance,
															shared_ptr<TVec> point2)
	{
		if (point != nullptr)
		{
			return TermBuilder::euclidianDistanceSqr(point, point2)
					> autodiff::TermBuilder::constant(distance * distance);
		}
		else
		{
			return nullptr;
		}
	}
	shared_ptr<Term> MSLConstraintBuilder::insideSphere(shared_ptr<TVec> centre, double distance,
														vector<shared_ptr<TVec>>& points)
	{
		if (centre != nullptr)
		{
			shared_ptr<Term> c = TermBuilder::euclidianDistance(centre, points[0])
					< autodiff::TermBuilder::constant(distance);
			for (int i = 1; i < points.size(); i++)
			{
				c = c & TermBuilder::euclidianDistance(centre, points[i]) < autodiff::TermBuilder::constant(distance);
			}
			return c;
		}
		else
		{
			return nullptr;
		}
	}

	shared_ptr<Term> MSLConstraintBuilder::insideSphere(shared_ptr<TVec> centre, double distance,
														shared_ptr<TVec> point)
	{
		if (centre != nullptr)
		{
			return TermBuilder::euclidianDistance(centre, point) < autodiff::TermBuilder::constant(distance);
		}
		else
		{
			return nullptr;
		}
	}

	shared_ptr<Term> MSLConstraintBuilder::outsideTriangle(shared_ptr<TVec> a, shared_ptr<TVec> b, shared_ptr<TVec> c,
															double tolerance, vector<shared_ptr<TVec>>& points)
	{
		shared_ptr<TVec> a2b = b - a;
		shared_ptr<TVec> b2c = c - b;
		shared_ptr<TVec> c2a = a - c;

		shared_ptr<Term> outsideConstraints = autodiff::LTConstraint::TRUE;
		shared_ptr<Term> pConsts;
		shared_ptr<TVec> a2p, b2p, c2p, p;
		for (int i = 0; i < points.size(); i++)
		{
			p = points[i];
			a2p = p - a;
			b2p = p - b;
			c2p = p - c;

			pConsts = ((a2p->getX() * a2b->getY() - a2p->getY() * a2b->getX())
					/ autodiff::TermBuilder::power(a2p->normSquared(), 0.5))
					> autodiff::TermBuilder::constant(tolerance);
			pConsts = pConsts
					| ((b2p->getX() * b2c->getY() - b2p->getY() * b2c->getX())
							/ autodiff::TermBuilder::power(b2p->normSquared(), 0.5))
							> autodiff::TermBuilder::constant(tolerance);
			pConsts = pConsts
					| ((c2p->getX() * c2a->getY() - c2p->getY() * c2a->getX())
							/ autodiff::TermBuilder::power(c2p->normSquared(), 0.5))
							> autodiff::TermBuilder::constant(tolerance);

			outsideConstraints = outsideConstraints & pConsts;
		}

		return outsideConstraints;
	}
	shared_ptr<Term> MSLConstraintBuilder::insideTriangle(shared_ptr<TVec> a, shared_ptr<TVec> b, shared_ptr<TVec> c,
															double tolerance, vector<shared_ptr<TVec>>& points)
	{
		shared_ptr<TVec> a2b = b - a;
		shared_ptr<TVec> b2c = c - b;
		shared_ptr<TVec> c2a = a - c;

		shared_ptr<Term> outsideConstraints = autodiff::LTConstraint::TRUE;
		shared_ptr<Term> pConsts;
		shared_ptr<TVec> a2p, b2p, c2p, p;
		for (int i = 0; i < points.size(); i++)
		{
			p = points[i];
			a2p = p - a;
			b2p = p - b;
			c2p = p - c;

			pConsts = ((a2p->getX() * a2b->getY() - a2p->getY() * a2b->getX())
					/ autodiff::TermBuilder::power(a2p->normSquared(), 0.5))
					< autodiff::TermBuilder::constant(-tolerance);
			pConsts = pConsts
					| ((b2p->getX() * b2c->getY() - b2p->getY() * b2c->getX())
							/ autodiff::TermBuilder::power(b2p->normSquared(), 0.5))
							< autodiff::TermBuilder::constant(-tolerance);
			pConsts = pConsts
					| ((c2p->getX() * c2a->getY() - c2p->getY() * c2a->getX())
							/ autodiff::TermBuilder::power(c2p->normSquared(), 0.5))
							< autodiff::TermBuilder::constant(-tolerance);

			outsideConstraints = outsideConstraints & pConsts;
		}

		return outsideConstraints;
	}
	shared_ptr<Term> MSLConstraintBuilder::insideKonvex(vector<shared_ptr<TVec>>& shell, double tolerance,
														vector<shared_ptr<TVec>>& points)
	{
		vector<shared_ptr<TVec>> shellVec;

		for (vector<shared_ptr<TVec>>::iterator ity = shell.begin(); ity != shell.end() - 1; ity++)
		{
			shellVec.push_back(*(ity + 1) - *ity);

		}
		shellVec.push_back(*shell.begin() - *shell.end());

		shared_ptr<Term> outsideConstraints = autodiff::LTConstraint::TRUE;
		shared_ptr<Term> pConsts;

	}

	shared_ptr<Term> MSLConstraintBuilder::outsideCakePiece(shared_ptr<TVec> a, shared_ptr<TVec> b, shared_ptr<TVec> c,
															double tolerance, vector<shared_ptr<TVec>>& points)
	{
		shared_ptr<TVec> a2b = b - a;
		shared_ptr<TVec> c2a = a - c;

		shared_ptr<Term> outsideConstraints = autodiff::LTConstraint::TRUE;
		shared_ptr<Term> pConsts;
		shared_ptr<TVec> a2p, b2p, c2p, p;
		for (int i = 0; i < points.size(); i++)
		{
			p = points[i];
			a2p = p - a;
			b2p = p - b;
			c2p = p - c;

			pConsts = ((a2p->getX() * a2b->getY() - a2p->getY() * a2b->getX())
					/ autodiff::TermBuilder::power(a2p->normSquared(), 0.5))
					> autodiff::TermBuilder::constant(tolerance);
			pConsts = pConsts
					| ((c2p->getX() * c2a->getY() - c2p->getY() * c2a->getX())
							/ autodiff::TermBuilder::power(c2p->normSquared(), 0.5))
							> autodiff::TermBuilder::constant(tolerance);

			outsideConstraints = outsideConstraints & pConsts;
		}

		return outsideConstraints;
	}
	shared_ptr<Term> MSLConstraintBuilder::applyRules(Situation situation, int specialIdx,
														vector<shared_ptr<TVec>>& fieldPlayers)
	{
		msl::MSLWorldModel* wm = msl::MSLWorldModel::get();
		shared_ptr<Term> c;
		shared_ptr<TVec> ballT = nullptr;
		auto ownPos = wm->rawSensorData->getOwnPositionVision();
		shared_ptr<geometry::CNPoint2D> ball = wm->ball->getEgoBallPosition();
		//		shared_ptr<geometry::CNPoint2D> ball = wm->ball.getEgoBallPosition()->egoToAllo(*wm->rawSensorData.getOwnPositionVision());
		if (ball != nullptr && ownPos != nullptr)
		{
			ball = ball->egoToAllo(*ownPos);
			ballT = make_shared<TVec>(initializer_list<double> {ball->x, ball->x});
		}
		switch (situation)
		{
			case Situation::Start:
			case Situation::Restart:
			case Situation::Ready:
			case Situation::FirstHalf:
			case Situation::SecondHalf:
				c = commonRules(fieldPlayers);
				break;
			case Situation::OwnCorner:
			case Situation::OwnFreekick:
			case Situation::OwnGoalkick:
			case Situation::OwnThrowin:
				c = ownStdRules(ballT, specialIdx, fieldPlayers);
				break;
			case Situation::OwnKickoff:
				c = ownKickOffRules(ballT, specialIdx, fieldPlayers);
				break;
			case Situation::OwnPenalty:
				c = ownPenaltyRules(ballT, specialIdx, fieldPlayers);
				break;
			case Situation::OppCorner:
			case Situation::OppFreekick:
			case Situation::OppGoalkick:
			case Situation::OppThrowin:
				c = oppStdRules(ballT, fieldPlayers);
				break;
			case Situation::OppKickoff:
				c = oppKickOffRules(ballT, fieldPlayers);
				break;
			case Situation::OppPenalty:
				c = oppPenaltyRules(fieldPlayers);
				break;
			case Situation::DropBall:
				c = dropBallRules(ballT, fieldPlayers);
				break;
			case Situation::Stop:
				// no constraints
				c = autodiff::LTConstraint::TRUE;
				break;
			default:
				throw new NoSituationFoundException(situation);
				break;
		}
		return c;
	}
	shared_ptr<Term> MSLConstraintBuilder::applyRules(int specialIdx, vector<shared_ptr<TVec>>& fieldPlayers)
	{
		msl::MSLWorldModel* wm = msl::MSLWorldModel::get();
		shared_ptr<Term> c;
		shared_ptr<TVec> ballT = nullptr;
		auto ownPos = wm->rawSensorData->getOwnPositionVision();
		shared_ptr<geometry::CNPoint2D> ball = wm->ball->getAlloBallPosition();
		if (ball != nullptr && ownPos != nullptr)
		{
			ballT = make_shared<TVec>(initializer_list<double> {ball->x, ball->y});
		}
		switch (wm->game->getSituation())
		{
			case Situation::Start:
			case Situation::Restart:
			case Situation::Ready:
			case Situation::FirstHalf:
			case Situation::SecondHalf:
				c = commonRules(fieldPlayers);
				break;
			case Situation::OwnCorner:
			case Situation::OwnFreekick:
			case Situation::OwnGoalkick:
			case Situation::OwnThrowin:
				c = ownStdRules(ballT, specialIdx, fieldPlayers);
				break;
			case Situation::OwnKickoff:
				c = ownKickOffRules(ballT, specialIdx, fieldPlayers);
				break;
			case Situation::OwnPenalty:
				c = ownPenaltyRules(ballT, specialIdx, fieldPlayers);
				break;
			case Situation::OppCorner:
			case Situation::OppFreekick:
			case Situation::OppGoalkick:
			case Situation::OppThrowin:
				c = oppStdRules(ballT, fieldPlayers);
				break;
			case Situation::OppKickoff:
				c = oppKickOffRules(ballT, fieldPlayers);
				break;
			case Situation::OppPenalty:
				c = oppPenaltyRules(fieldPlayers);
				break;
			case Situation::DropBall:
				c = dropBallRules(ballT, fieldPlayers);
				break;
			case Situation::Stop:
				// no constraints
				c = autodiff::LTConstraint::TRUE;
				break;
			default:
				throw new NoSituationFoundException(wm->game->getSituation());
				break;
		}
		return c;
	}
	shared_ptr<Term> MSLConstraintBuilder::commonRules(vector<shared_ptr<TVec>>& fieldPlayers)
	{
		shared_ptr<Term> appliedRules = spread(MIN_POSITION_DIST, fieldPlayers);
		appliedRules = appliedRules & ownPenaltyAreaRule(fieldPlayers);
		appliedRules = appliedRules & oppPenaltyAreaRule(fieldPlayers);
		appliedRules = appliedRules & outsideArea(Areas::OwnGoalArea, fieldPlayers);
		appliedRules = appliedRules & outsideArea(Areas::OppGoalArea, fieldPlayers);
		return appliedRules;
	}
	shared_ptr<Term> MSLConstraintBuilder::dropBallRules(shared_ptr<TVec> ballT, vector<shared_ptr<TVec>>& fieldPlayers)
	{
		shared_ptr<Term> appliedRules = spread(MIN_POSITION_DIST, fieldPlayers);
		appliedRules = appliedRules & outsideArea(Areas::OwnGoalArea, fieldPlayers);
		appliedRules = appliedRules & outsideArea(Areas::OppGoalArea, fieldPlayers);
		appliedRules = appliedRules & ownPenaltyAreaRule(fieldPlayers);
		appliedRules = appliedRules & oppPenaltyAreaRule(fieldPlayers);
		if (ballT != nullptr)
		{
			appliedRules = appliedRules & outsideSphere(ballT, rules->getStayAwayRadiusDropBall(), fieldPlayers);
		}
		return appliedRules;
	}
	shared_ptr<Term> MSLConstraintBuilder::ownPenaltyRules(shared_ptr<TVec> ballT, int executerIdx,
															vector<shared_ptr<TVec>>& fieldPlayers)
	{
		shared_ptr<Term> appliedRules = spread(MIN_POSITION_DIST, fieldPlayers);
		appliedRules = appliedRules
				& alica::ConstraintBuilder::equals(fieldPlayers[executerIdx], centreMarkT, ON_LINE_TOL);
		vector<shared_ptr<TVec>> passivPlayers;
		std::copy_if(std::begin(fieldPlayers), std::end(fieldPlayers), std::back_inserter(passivPlayers),
						[&](shared_ptr<TVec> v)
						{	return v != fieldPlayers[executerIdx];});

//		TVec[] passivPlayers = Array.FindAll (fieldplayers, val => val != fieldplayers [executerIdx]).ToArray ();

		if (ballT != nullptr)
		{
			appliedRules = appliedRules
					& alica::ConstraintBuilder::ifThenElse(ballT->getX() > (autodiff::TermBuilder::constant(1000.0)),
															insideArea(Areas::OwnHalf, passivPlayers),
															insideArea(Areas::OppHalf, passivPlayers));
		}
		else
		{
			appliedRules = appliedRules & insideArea(Areas::OwnHalf, passivPlayers);
		}
		appliedRules = appliedRules & outsideSphere(centreMarkT, rules->getStayAwayRadius(), passivPlayers);
		return appliedRules;
	}
	shared_ptr<Term> MSLConstraintBuilder::oppPenaltyRules(vector<shared_ptr<TVec>>& fieldPlayers)
	{
		shared_ptr<Term> appliedRules = insideArea(Areas::OppHalf, fieldPlayers);
		appliedRules = appliedRules & outsideSphere(centreMarkT, rules->getStayAwayRadius(), fieldPlayers);
		appliedRules = appliedRules & spread(MIN_POSITION_DIST, fieldPlayers);
		return appliedRules;
	}
	shared_ptr<Term> MSLConstraintBuilder::ownKickOffRules(shared_ptr<TVec> ballT, int executerIdx,
															vector<shared_ptr<TVec>>& fieldPlayers)
	{
		shared_ptr<Term> appliedRules = spread(MIN_POSITION_DIST, fieldPlayers);
		appliedRules = appliedRules & outsideArea(Areas::OwnGoalArea, fieldPlayers);

		vector<shared_ptr<TVec>> receivers;
		std::copy_if(std::begin(fieldPlayers), std::end(fieldPlayers), std::back_inserter(receivers),
						[&](shared_ptr<TVec> v)
						{	return v != fieldPlayers[executerIdx];});

		if (ballT != nullptr)
		{
			appliedRules = appliedRules & outsideSphere(ballT, rules->getStayAwayRadius(), receivers);
		}
		else
		{
			appliedRules = appliedRules & outsideSphere(centreMarkT, rules->getStayAwayRadius(), receivers);
		}
		appliedRules = appliedRules & ownPenaltyAreaRule(receivers);
		appliedRules = appliedRules & insideArea(Areas::OwnHalf, receivers);
		return appliedRules;
	}
	shared_ptr<Term> MSLConstraintBuilder::oppKickOffRules(shared_ptr<TVec> ballT,
															vector<shared_ptr<TVec>>& fieldPlayers)
	{
		shared_ptr<Term> appliedRules = spread(MIN_POSITION_DIST, fieldPlayers);
		appliedRules = appliedRules & outsideArea(Areas::OwnGoalArea, fieldPlayers);
		if (ballT != nullptr)
		{
			appliedRules = appliedRules & outsideSphere(ballT, rules->getStayAwayRadiusOpp(), fieldPlayers);
		}
		else
		{
			appliedRules = appliedRules & outsideSphere(centreMarkT, rules->getStayAwayRadiusOpp(), fieldPlayers);
		}
		appliedRules = appliedRules & ownPenaltyAreaRule(fieldPlayers);
		appliedRules = appliedRules & insideArea(Areas::OwnHalf, fieldPlayers);
		return appliedRules;
	}
	shared_ptr<Term> MSLConstraintBuilder::ownStdRules(shared_ptr<TVec> ballT, int executerIdx,
														vector<shared_ptr<TVec>>& fieldPlayers)
	{
		shared_ptr<Term> appliedRules = spread(MIN_POSITION_DIST, fieldPlayers);
		appliedRules = appliedRules & outsideArea(Areas::OwnGoalArea, fieldPlayers);
		appliedRules = appliedRules & outsideArea(Areas::OppGoalArea, fieldPlayers);
		vector<shared_ptr<TVec>> receivers;
		if (executerIdx >= 0)
		{
			std::copy_if(std::begin(fieldPlayers), std::end(fieldPlayers), std::back_inserter(receivers),
							[&](shared_ptr<TVec> v)
							{	return v != fieldPlayers[executerIdx];});
		}
		else
		{
			receivers = fieldPlayers;
		}
		if (ballT != nullptr)
		{
			appliedRules = appliedRules & outsideSphere(ballT, rules->getStayAwayRadius(), receivers);
		}
		appliedRules = appliedRules & ownPenaltyAreaRule(receivers);
		appliedRules = appliedRules & oppPenaltyAreaRule(receivers);
		return appliedRules;
	}
	shared_ptr<Term> MSLConstraintBuilder::oppStdRules(shared_ptr<TVec> ballT, vector<shared_ptr<TVec>>& fieldPlayers)
	{

		shared_ptr<Term> appliedRules = outsideArea(Areas::OwnGoalArea, fieldPlayers);
		appliedRules = appliedRules & outsideArea(Areas::OppGoalArea, fieldPlayers);
		appliedRules = appliedRules & ownPenaltyAreaDistanceExceptionRule(ballT, fieldPlayers);
		appliedRules = appliedRules & oppPenaltyAreaRule(fieldPlayers);
		appliedRules = appliedRules & spread(1000.0, fieldPlayers);
		return appliedRules;
	}
	shared_ptr<Term> MSLConstraintBuilder::ownPenaltyAreaDistanceExceptionRule(shared_ptr<TVec> ballT,
																				vector<shared_ptr<TVec>>& fieldPlayers)
	{
		shared_ptr<Term> ownPenaltyConstrains = autodiff::Term::FALSE;
		shared_ptr<Term> minDistConstrains = autodiff::Term::TRUE;
		vector<shared_ptr<TVec>> notInPenaltyPlayers;
		for (int i = 0; i < fieldPlayers.size(); i++)
		{
			notInPenaltyPlayers.clear();
			for (int j = 0; j < fieldPlayers.size(); j++)
			{
				if (i != j)
					notInPenaltyPlayers.push_back(fieldPlayers[j]);
			}
			if (notInPenaltyPlayers.size() > 0)
				ownPenaltyConstrains = ownPenaltyConstrains | outsideArea(Areas::OwnPenaltyArea, notInPenaltyPlayers);
			if (ballT != nullptr)
			{
				minDistConstrains = minDistConstrains
						& alica::ConstraintBuilder::ifThen(
								outsideArea(Areas::OwnPenaltyArea, fieldPlayers.at(i)),
								outsideSphere(ballT, rules->getStayAwayRadiusOpp(), fieldPlayers.at(i)));
			}
		}

		if (ownPenaltyConstrains != autodiff::Term::FALSE)
			return ownPenaltyConstrains & minDistConstrains;
		else
			return minDistConstrains;
	}
	shared_ptr<Term> MSLConstraintBuilder::ownPenaltyAreaRule(vector<shared_ptr<TVec>>& fieldPlayers)
	{
		if (fieldPlayers.size() <= 1)
			return autodiff::Term::TRUE;
		shared_ptr<Term> ownPenaltyConstrains = autodiff::Term::FALSE;
		vector<shared_ptr<TVec>> notInPenaltyPlayers;
		int index = 0;
		for (int i = 0; i < fieldPlayers.size(); i++)
		{
			index = 0;
			for (int j = 0; j < fieldPlayers.size(); j++)
			{
				if (i != j)
					notInPenaltyPlayers.push_back(fieldPlayers[j]);
			}
			ownPenaltyConstrains = ownPenaltyConstrains | outsideArea(Areas::OwnPenaltyArea, notInPenaltyPlayers);
		}
		return ownPenaltyConstrains;
	}
	shared_ptr<Term> MSLConstraintBuilder::oppPenaltyAreaRule(vector<shared_ptr<TVec>>& fieldPlayers)
	{
		if (fieldPlayers.size() <= 1)
			return autodiff::Term::TRUE;
		shared_ptr<Term> oppPenaltyConstrains = autodiff::Term::FALSE;
		vector<shared_ptr<TVec>> notInPenaltyPlayers;
		int index = 0;
		for (int i = 0; i < fieldPlayers.size(); i++)
		{
			notInPenaltyPlayers.clear();
			index = 0;
			for (int j = 0; j < fieldPlayers.size(); j++)
			{
				if (i != j)
					notInPenaltyPlayers.push_back(fieldPlayers[j]);
			}
			if (notInPenaltyPlayers.size() > 0)
				oppPenaltyConstrains = oppPenaltyConstrains | outsideArea(Areas::OwnPenaltyArea, notInPenaltyPlayers);
		}
		return oppPenaltyConstrains;
	}

	shared_ptr<Term> MSLConstraintBuilder::outsideArea(Areas area, shared_ptr<TVec> point)
	{
		vector<shared_ptr<TVec>> points;
		points.push_back(point);
		return outsideArea(area, points);
	}

	shared_ptr<Term> MSLConstraintBuilder::outsideArea(Areas area, vector<shared_ptr<TVec>>& points)
	{
		shared_ptr<geometry::CNPoint2D> lowerRightCornerP;
		shared_ptr<geometry::CNPoint2D> upperLeftCornerP;
		resolveArea(area, &lowerRightCornerP, &upperLeftCornerP);
		shared_ptr<TVec> lowerRightCorner = make_shared<TVec>(
				initializer_list<double> {lowerRightCornerP->x - AREA_TOL, lowerRightCornerP->y - AREA_TOL});
		shared_ptr<TVec> upperLeftCorner = make_shared<TVec>(
				initializer_list<double> {upperLeftCornerP->x + AREA_TOL, upperLeftCornerP->y + AREA_TOL});
		return outsideRectangle(lowerRightCorner, upperLeftCorner, points);
	}

	shared_ptr<Term> MSLConstraintBuilder::insideArea(Areas area, shared_ptr<TVec> point)
	{

		vector<shared_ptr<TVec>> points;
		points.push_back(point);
		return insideArea(area, points);
	}

	shared_ptr<Term> MSLConstraintBuilder::insideArea(Areas area, vector<shared_ptr<TVec>>& points)
	{
		if (points.size() <= 0)
			return autodiff::Term::TRUE;
		shared_ptr<geometry::CNPoint2D> lowerRightCornerP;
		shared_ptr<geometry::CNPoint2D> upperLeftCornerP;
		resolveArea(area, &lowerRightCornerP, &upperLeftCornerP);
		shared_ptr<TVec> lowerRightCorner = make_shared<TVec>(
				initializer_list<double> {lowerRightCornerP->x - AREA_TOL, lowerRightCornerP->y - AREA_TOL});
		shared_ptr<TVec> upperLeftCorner = make_shared<TVec>(
				initializer_list<double> {upperLeftCornerP->x + AREA_TOL, upperLeftCornerP->y + AREA_TOL});
		return insideRectangle(lowerRightCorner, upperLeftCorner, points);
	}

	//	shared_ptr<geometry::CNPoint2D> MSLConstraintBuilder::oppLeftSurCornerP = wm->field.posULSurrounding();

	void MSLConstraintBuilder::resolveArea(Areas area, shared_ptr<geometry::CNPoint2D> *lowerRightCorner,
											shared_ptr<geometry::CNPoint2D> *upperLeftCorner)
	{
		MSLWorldModel* wm = MSLWorldModel::get();
		switch (area)
		{
			case Areas::Surrounding:
				*lowerRightCorner = wm->field->posLRSurrounding();
				//TODO was *upperLeftCorner = wm->field.posLeftOppCorner(); but should be with surrounding
				*upperLeftCorner = wm->field->posULSurrounding();
				break;
			case Areas::Field:
				*lowerRightCorner = wm->field->posRightOwnCorner();
				*upperLeftCorner = wm->field->posLeftOppCorner();
				break;
			case Areas::OppHalf:
				*lowerRightCorner = wm->field->posLROppHalf();
				*upperLeftCorner = wm->field->posLeftOppCorner();
				break;
			case Areas::OwnHalf:
				*lowerRightCorner = wm->field->posRightOwnCorner();
				*upperLeftCorner = wm->field->posULOwnHalf();
				break;
			case Areas::OppPenaltyArea:
				*lowerRightCorner = wm->field->posLROppPenaltyArea();
				*upperLeftCorner = wm->field->posULOppPenaltyArea();
				break;
			case Areas::OwnPenaltyArea:
				*lowerRightCorner = wm->field->posLROwnPenaltyArea();
				*upperLeftCorner = wm->field->posULOwnPenaltyArea();
				break;
			case Areas::OwnGoalArea:
				*lowerRightCorner = wm->field->posLROwnGoalArea();
				*upperLeftCorner = wm->field->posULOwnGoalArea();
				break;
			case Areas::OppGoalArea:
				*lowerRightCorner = wm->field->posLROppGoalArea();
				*upperLeftCorner = wm->field->posULOppGoalArea();
				break;
			default:
				throw "Unknown Area!";
		}
	}
} /* namespace msl */
