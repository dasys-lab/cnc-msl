using namespace std;
#include "Plans/Behaviours/Duel.h"

/*PROTECTED REGION ID(inccpp1450178699265) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1450178699265) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	Duel::Duel() :
			DomainBehaviour("Duel")
	{
		/*PROTECTED REGION ID(con1450178699265) ENABLED START*/ //Add additional options here
		wheelSpeed = (*this->sc)["Actuation"]->get<double>("Dribble.DuelWheelSpeed", NULL);
		translation = (*this->sc)["Drive"]->get<double>("Drive.Duel.Velocity", NULL);
		fieldLength = (*this->sc)["Globals"]->get<double>("Globals.FootballField.FieldLength", NULL);
		fieldWidth = (*this->sc)["Globals"]->get<double>("Globals.FootballField.FieldWidth", NULL);
		robotRadius = (*this->sc)["Rules"]->get<double>("Rules.RobotRadius", NULL);
		radiusToCheckOpp = 2000.0;
		radiusToCheckOwn = 2000.0;
		timeToStayInDuel = 9000000000;
		/*PROTECTED REGION END*/
	}
	Duel::~Duel()
	{
		/*PROTECTED REGION ID(dcon1450178699265) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void Duel::run(void* msg)
	{
		/*PROTECTED REGION ID(run1450178699265) ENABLED START*/ //Add additional options here
		// enter plan when !haveBall && enemy haveBall || haveBall && enemy close
		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData.getOwnPositionVision();
		shared_ptr<geometry::CNPoint2D> egoBallPos = wm->ball.getEgoBallPosition();
		shared_ptr<geometry::CNPoint2D> egoTarget = nullptr;
		shared_ptr<geometry::CNPoint2D> egoAlignPoint = make_shared<geometry::CNPoint2D>(fieldLength / 2, 0)->alloToEgo(
				*ownPos);
//		shared_ptr<geometry::CNPoint2D> oppGoal = make_shared<geometry::CNPoint2D>(fieldLength / 2, 0);
		shared_ptr<geometry::CNPoint2D> oppGoal = msl::MSLFootballField::posOppGoalMid();
		msl_actuator_msgs::MotionControl mc;
		msl_actuator_msgs::BallHandleCmd bhc;

		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> teamMatePositions = make_shared<
		vector<shared_ptr<geometry::CNPoint2D>>>();
		shared_ptr<geometry::CNPoint2D> closestFriendly = nullptr;

		if (ownPos == nullptr || egoBallPos == nullptr)
		{
			return;
		}

		if (usedBallPos == nullptr
				|| (egoBallPos->egoToAllo(*ownPos) - usedBallPos->egoToAllo(*ownPos))->length() > 200)
		{
			usedBallPos = egoBallPos;
		}

		shared_ptr<geometry::CNPoint2D> ballOrth1 = make_shared<geometry::CNPoint2D>(egoBallPos->y, -egoBallPos->x);
		shared_ptr<geometry::CNPoint2D> ballOrth2 = make_shared<geometry::CNPoint2D>(-egoBallPos->y, egoBallPos->x);

//		shared_ptr<geometry::CNPoint2D> ballOrth1 = make_shared<geometry::CNPoint2D>(usedBallPos->y, -usedBallPos->x);
//		shared_ptr<geometry::CNPoint2D> ballOrth2 = make_shared<geometry::CNPoint2D>(-usedBallPos->y, usedBallPos->x);

		shared_ptr<geometry::CNPoint2D> ownPoint = make_shared<geometry::CNPoint2D>(ownPos->x, ownPos->y);

		// push enemy robot and try to take the ball
		if (!wm->ball.haveBall()
				|| (wm->ball.haveBall() && wm->game.getGameState() == msl::GameState::OppBallPossession))
		{
			//mc.motion.translation = 2 * translation;
			mc.motion.translation = translation;
			mc.motion.rotation = egoBallPos->rotate(M_PI)->angleTo() * 1.8;
			mc.motion.angle = egoBallPos->angleTo();

			bhc.leftMotor = -wheelSpeed;
			bhc.rightMotor = -wheelSpeed;

			send(bhc);
			send(mc);

		}
		else
		{
			shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> mostRecentOpps =
			wm->robots.opponents.getOpponentsAlloClustered();
			shared_ptr<geometry::CNPoint2D> closestOpponent = nullptr;

			if (mostRecentOpps != nullptr)
			{
				for (auto opp : *mostRecentOpps)
				{
					if (opp)
					{

						//find closest opponent in 2m radius to determine direction to move towards
						if ((closestOpponent == nullptr
										|| opp->distanceTo(ownPoint) < closestOpponent->distanceTo(ownPoint))
								&& opp->distanceTo(ownPoint) < radiusToCheckOpp)
						{
							closestOpponent = opp;
						}

						teamMatePositions = wm->robots.teammates.getTeammatesAlloClustered();

						// try to find closest team member that isn't blocked for aligning
						for (int i = 0; i < teamMatePositions->size(); i++)
						{
							auto pos = teamMatePositions->at(i);
							if (pos)
							{
								shared_ptr<geometry::CNPoint2D> friendlyPos = make_shared<geometry::CNPoint2D>(pos->x,
										pos->y);

								//determine points for corridor check
								shared_ptr<geometry::CNPoint2D> friendlyOrth1 = make_shared<geometry::CNPoint2D>(
										friendlyPos->y, -friendlyPos->x);
								shared_ptr<geometry::CNPoint2D> friendlyOrth2 = make_shared<geometry::CNPoint2D>(
										-friendlyPos->y, friendlyPos->x);

								shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> trianglePoints = make_shared<
								vector<shared_ptr<geometry::CNPoint2D>>>();

								trianglePoints->push_back(
										friendlyPos + friendlyOrth1->normalize() * (robotRadius + 75));
								trianglePoints->push_back(
										friendlyPos + friendlyOrth2->normalize() * (robotRadius + 75));
								trianglePoints->push_back(ownPoint);

								//check if opponent stands between me and my teammate
								if (geometry::isInsidePolygon(*trianglePoints, opp))
								{
									friendlyBlocked = true;
								}

								//if (friendlyPos->distanceTo(ownPoint) < 2000 && !friendlyBlocked)
								//cout << "Duel: distance to friendlyPos: " << friendlyPos->distanceTo(ownPoint) << endl;

								// find closest team member in 2m radius
								if (friendlyPos->distanceTo(ownPoint) < radiusToCheckOwn)
								{
									if (closestFriendly == nullptr
											|| closestFriendly->distanceTo(ownPoint)
											< friendlyPos->distanceTo(ownPoint))
									{
										closestFriendly = friendlyPos;
									}
								}
							}

						}
					}

				}

			}

			if (closestOpponent != nullptr)
			{
				hadClosestOpp.at(itcounter++ % 10) = true;
				itcounter++;

				//TODO cooleren punkt berechnen?

//					egoTarget = (ownPoint + (oppGoal + ballOrth1)->normalize() * 1000)->alloToEgo(*ownPos);
//					egoTarget = (closestOpponent->alloToEgo(*ownPos)->rotate(M_PI) + oppGoal->alloToEgo(*ownPos))->normalize()*1000;
				egoTarget = (closestOpponent->alloToEgo(*ownPos) + oppGoal->alloToEgo(*ownPos))->normalize()*2000;

//					egoTarget = (ownPoint + (oppGoal + ballOrth2)->normalize() * 1000)->alloToEgo(*ownPos);

				// move away from opponent
				// egoTarget = (ownPoint + closestOpponent->rotate(M_PI)->normalize() * 300)->alloToEgo(*ownPos);

				//egoTarget = (ownPoint + ((ownPoint - closestOpponent)->normalize() * 300))->alloToEgo(*ownPos);

			}
			else
			{

				hadClosestOpp.at(itcounter % 10) = false;
				itcounter++;

				//TODO hysteresis of last closest opps because sensor info might be faulty??

				//attempt at hysteresis , not sure if working
				int counter = 0;

				for (int i = 0; i < hadClosestOpp.size(); i++)
				{
					if (hadClosestOpp.at(i))
					{
						counter += hadClosestOpp.size() - i;
					}
					else
					{
						counter -= hadClosestOpp.size() - i;
					}
				}

				if (counter <= 0)
				{
					cout << "Duel: Success, far away from opponent" << endl;
					this->success = true;
					return;
				}

			}
			if (closestFriendly != nullptr && !friendlyBlocked)
			{
				//align to non-blocked closest team member for future pass play
				egoAlignPoint = closestFriendly->alloToEgo(*ownPos);
			}
			else if (closestFriendly != nullptr && friendlyBlocked)
			{

				//can't align to team member so align to opp goal
				egoAlignPoint = oppGoal->alloToEgo(*ownPos);
			}
			else
			{
				//found no team member at all
				cout << "Duel: Found nobody" << endl;
				if (ownPos == nullptr)
				{
					//no idea
					cout << "Duel: no idea" << endl;
					egoAlignPoint = oppGoal->alloToEgo(*ownPos);
				}
				else
				{
					//try closest field border

					//TODO testen
					cout << "Duel: try closest field border" << endl;

					//shared_ptr < geometry::CNPoint2D > ballOrth1 = make_shared < geometry::CNPoint2D
					//> (egoBallPos->y, -egoBallPos->x);
					//shared_ptr < geometry::CNPoint2D > ballOrth2 = make_shared < geometry::CNPoint2D
					//> (-egoBallPos->y, egoBallPos->x);
					//ballOrth1 = ballOrth1->egoToAllo(*ownPos);
					//ballOrth2 = ballOrth1->egoToAllo(*ownPos);

					double distance = msl::MSLFootballField::distanceToLine(ownPoint,
							ballOrth1->egoToAllo(*ownPos)->angleTo());
					if (msl::MSLFootballField::distanceToLine(ownPoint, ballOrth2->egoToAllo(*ownPos)->angleTo())
							< distance)
					{
						//top line
						egoAlignPoint = make_shared<geometry::CNPoint2D>(ownPoint->x, -fieldWidth/2)->alloToEgo(*ownPos);
					}
					else
					{
						//bottom line
						egoAlignPoint = make_shared<geometry::CNPoint2D>(ownPoint->x, fieldWidth/2)->alloToEgo(*ownPos);
					}
				}
			}

			//cout << "Duel: Moving away!" << endl;
			//TODO does moveToPointFast work??
			if(!egoTarget)
			{
				egoTarget = (make_shared<geometry::CNPoint2D>(0,0))->alloToEgo(*ownPos);
			}

			mc = msl::RobotMovement::moveToPointCarefully(egoTarget, egoAlignPoint, 100);
			send(mc);
		}

		// too much time has passed, we don't want to stay in duel for too long (rules says sth like 10s until ball dropped)
		if (wm->getTime() - entryTime > timeToStayInDuel)
		{
			cout << "Duel: time over " << endl;
			this->success = true;
		}

		//		shared_ptr<geometry::CNPoint2D> ownGoalPos = make_shared<geometry::CNPoint2D>(-fieldLength / 2, 0.0);
		//		shared_ptr<geometry::CNPoint2D> goalPosEgo = ownGoalPos->alloToEgo(*me);
		//
		//		if (ownGoalPos != nullptr && ownPoint->distanceTo(ownGoalPos) < fieldLength / 3)
		//		{
		//			cout << "Duel: own goal is close" << endl;
		//			//own goal is close, get the ball away
		//			if (pointLeftOfVec(egoBallPos, goalPosEgo))
		//			{
		//				cout << "Goal left!" << endl;
		//				// goal is on the left side of a vector between me and the ball, so i turn right
		//				egoAlignPoint = make_shared<geometry::CNPoint2D>(ownPoint->x, -fieldWidth / 2);
		//
		//			}
		//			else
		//			{
		//
		//				cout << "Goal right!" << endl;
		//				//goal on right side, turn left!
		//				egoAlignPoint = make_shared<geometry::CNPoint2D>(ownPoint->x, fieldWidth / 2);
		//				egoTarget = make_shared<geometry::CNPoint2D>(0, fieldWidth / 2);
		//			}
		//		}

		/*PROTECTED REGION END*/
	}
	void Duel::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1450178699265) ENABLED START*/ //Add additional options here
		direction = 0;
		friendlyBlocked = false;
		entryTime = wm->getTime();
		itcounter = 0;
		hadClosestOpp = vector<bool>(10, true);
		usedBallPos = nullptr;
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1450178699265) ENABLED START*/ //Add additional methods here
// returns true if pointToCheck is left of lineVector(look along the lineVector)
// uses cross product of 2 vectors. 0: colinear, <0: point left of vec, >0: point right of vec
	bool Duel::pointLeftOfVec(shared_ptr<geometry::CNPoint2D> lineVector, shared_ptr<geometry::CNPoint2D> pointToCheck)
	{
		double cross = pointToCheck->x * lineVector->y - pointToCheck->y * lineVector->x;
		return (cross < 0);
	}

/*PROTECTED REGION END*/
} /* namespace alica */
