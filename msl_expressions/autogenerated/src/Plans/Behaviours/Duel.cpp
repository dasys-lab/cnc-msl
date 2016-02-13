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

		shared_ptr<geometry::CNPosition> me = wm->rawSensorData.getOwnPositionVision();
		shared_ptr<geometry::CNPoint2D> ownPoint = make_shared<geometry::CNPoint2D>(me->x, me->y);
		shared_ptr<geometry::CNPoint2D> egoBallPos = wm->ball.getEgoBallPosition();
		shared_ptr<geometry::CNPoint2D> egoAlignPoint = nullptr;
		msl_actuator_msgs::MotionControl mc;

		if (me == nullptr || egoBallPos == nullptr)
		{
			return;
		}

		shared_ptr<geometry::CNPoint2D> alloBallPos = egoBallPos->alloToEgo(*me);
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<
		vector<shared_ptr<geometry::CNPoint2D>>>();

		additionalPoints->push_back(alloBallPos);

		msl_actuator_msgs::BallHandleCmd bhc;
		bhc.leftMotor = -wheelSpeed;
		bhc.rightMotor = -wheelSpeed;
		send(bhc);

		//quickly drive toward the ball for a few seconds
		//TODO improve this and set itCounter accordingly or check for haveball

//		if (itCounter++ < 200)
//		{
//
//			mc.motion.angle = egoBallPos->angleTo();
//			mc.motion.rotation = 0;
//			mc.motion.translation = 2 * translation;
//			send(mc);
//			return;
//		}

		shared_ptr<geometry::CNPoint2D> ownGoalPos = make_shared<geometry::CNPoint2D>(-fieldLength / 2, 0.0);
		shared_ptr<geometry::CNPoint2D> goalPosEgo = ownGoalPos->alloToEgo(*me);
		//TODO direction == 0;

		if (!alignPointSet)
		{
			// distanceTo??
			if (ownGoalPos != nullptr && ownPoint->distanceTo(ownGoalPos) < fieldLength / 3)
			{

				cout << "Duel: goal is close" << endl;

				//own goal is close, get the ball away

				if (checkSide(egoBallPos, goalPosEgo))
				{
					// goal is on the left side of a vector between me and the ball, so i turn right
//					direction = -1;
					egoAlignPoint = egoBallPos->rotate(-M_PI / 2);
				}
				else
				{
					//goal on right side, turn left!
//					direction = 1;
					egoAlignPoint = egoBallPos->rotate(M_PI / 2);
				}
			}
			else
			{

				cout << "Duel: Goal far away" << endl;

				//own goal is far away, look for nearby friends

				shared_ptr<geometry::CNPoint2D> friendly = nullptr;
				shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> teamMatePositions = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();;
				int ownID = this->sc->getOwnRobotID();

				//mops 1, hairy 8, nase 9, savvy 10, myo 11
				//TODO dangerous on ID changes :(

				vector<int> teamMateIDs = vector<int> {8, 9, 10, 11};
				for (int id : teamMateIDs)
				{
					if (wm->robots.getTeamMatePosition(id, 0) != nullptr && id != ownID)
					{
						shared_ptr<geometry::CNPoint2D> validPosition = make_shared<geometry::CNPoint2D>(
								wm->robots.getTeamMatePosition(id, 0)->x, wm->robots.getTeamMatePosition(id, 0)->y);
						cout << "Duel: pushing back team mate position of tm w/ tm8id " << id << endl;
						teamMatePositions->push_back(validPosition);
					}

					cout << "Duel: Size of teamMatePositions vector: " << teamMatePositions->size() << endl;

					for (int i = 0; i < teamMatePositions->size(); i++)
					{

						auto pos = teamMatePositions->at(i);
						if (pos != nullptr)
						{
							cout << "Duel: Positions of Teammates: X= " << pos->x << " Y= " << pos->y << endl;
							cout << "Duel: Own Position: X= " << ownPoint->x << "Y= " << ownPoint->y << endl;

							shared_ptr<geometry::CNPoint2D> friendlyPos = make_shared<geometry::CNPoint2D>(pos->x,
																											pos->y);

							if (friendlyPos->distanceTo(ownPoint) < 2000)
							{
								if (friendly == nullptr
										|| friendly->distanceTo(ownPoint) < friendlyPos->distanceTo(ownPoint))
								{
									friendly = friendlyPos;
								}
							}

							if (friendly != nullptr)
							{
								//found one, try to get the ball to him
								if (checkSide(egoBallPos, friendly) && !alignPointSet)
								{
									cout << "Friendly is left of me and ball!" << endl;
//									egoAlignPoint = egoBallPos->rotate(M_PI / 2);
									egoAlignPoint = friendly;
									alignPointSet = true;
								}
								else if (!checkSide(egoBallPos, friendly) && !alignPointSet)
								{
									cout << "Friendly is right of me and ball!" << endl;
//									egoAlignPoint = egoBallPos->rotate(-M_PI / 2);
									egoAlignPoint = friendly;
									alignPointSet = true;
								}
							}

							else if (me != nullptr)
							{
								bool posFree = true;
								bool negFree = true;
								//found none, looking for free space

								//TODO anpassen sobald wir eine liste von gegnern im WM haben

								//TODO schleife fixen

								for (auto it = wm->robots.getObstaclePoints(0)->begin();
										it != wm->robots.getObstaclePoints(0)->end(); it++)
								{
									//TODO friendly darf nicht obstacle sein

									if (it->get() != nullptr)
									{
										cout << "Duel: it.get: " << it->get()->x << ", " << it->get()->y << endl;

										shared_ptr<geometry::CNPoint2D> obs = make_shared<geometry::CNPoint2D>(
												it->get()->x, it->get()->y);
										shared_ptr<geometry::CNPoint2D> egoObs = obs->egoToAllo(*me);

										if (obs->distanceTo(alloBallPos) < 2000 && fabs(obs->angleTo()) < M_PI / 3 * 2)
										{
											if (checkSide(egoBallPos, obs))
											{
												posFree = false;
												cout << "Obstacle left of me and the ball!" << endl;
											}
											else
											{
												negFree = false;
												cout << "Obstacle right of me and the ball!" << endl;

											}
										}
									}

								}
								if (posFree)
								{
									egoAlignPoint = egoBallPos->rotate(M_PI / 2);
								}
								else if (negFree)
								{
									egoAlignPoint = egoBallPos->rotate(-M_PI / 2);
								}
								else
								{
									//all occupied
									if (me == nullptr)
									{
										//no idea
										cout << "Duel: no idea" << endl;
										egoAlignPoint = egoBallPos->rotate(M_PI / 2);
									}
									else
									{

										cout << "Duel: try closest field border" << endl;

										shared_ptr<geometry::CNPoint2D> ballOrth1 = make_shared<geometry::CNPoint2D>(
												egoBallPos->y, -egoBallPos->x);
										shared_ptr<geometry::CNPoint2D> ballOrth2 = make_shared<geometry::CNPoint2D>(
												-egoBallPos->y, egoBallPos->x);
										ballOrth1 = ballOrth1->egoToAllo(*me);
										ballOrth2 = ballOrth1->egoToAllo(*me);
										double distance = distanceToFieldBorder(ownPoint, ballOrth1->angleTo());
										if (distanceToFieldBorder(ownPoint, ballOrth2->angleTo()) < distance)
										{
											egoAlignPoint = egoBallPos->rotate(M_PI / 2);
										}
										else
										{
											egoAlignPoint = egoBallPos->rotate(-M_PI / 2);
										}

									}
								}
							}
						}
					}
				}
			}

			mc = msl::RobotMovement::moveToPointCarefully(egoBallPos + egoBallPos->normalize() * 30, egoAlignPoint, 0);

//		mc.motion.angle = egoBallPos->angleTo() + direction * M_PI;
//		mc.motion.rotation = direction * M_PI / 2.0;
//		mc.motion.translation = translation;

			send(mc);

			//TODO issues:
			//wrong direction when goal close??
			//test checkSide

			// exit plan when haveBall && enemy not close

			/*PROTECTED REGION END*/
		}
	}
	void Duel::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1450178699265) ENABLED START*/ //Add additional options here
		itCounter = 0;
		direction = 0;
		alignPointSet = false;
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1450178699265) ENABLED START*/ //Add additional methods here
// returns true if pointToCheck is left of lineVector(look along the lineVector)
// uses cross product of 2 vectors. 0: colinear, <0: point left of vec, >0: point right of vec
	bool Duel::checkSide(shared_ptr<geometry::CNPoint2D> lineVector, shared_ptr<geometry::CNPoint2D> pointToCheck)
	{
		double cross = pointToCheck->x * lineVector->y - pointToCheck->y * lineVector->x;
		return (cross < 0);
	}

	double Duel::distanceToFieldBorder(shared_ptr<geometry::CNPoint2D> point, double angle)
	{

		shared_ptr<geometry::CNPoint2D> fieldCorner1 = make_shared<geometry::CNPoint2D>(-fieldLength / 2,
																						fieldWidth / 2);
		shared_ptr<geometry::CNPoint2D> fieldCorner2 = make_shared<geometry::CNPoint2D>(fieldLength / 2,
																						-fieldWidth / 2);

		double distance = 100000;
		double d;

		double vx = cos(angle);
		double vy = sin(angle);

		//Vector starts at (pointX,pointY), following it along a distance d gets you to (pointX + d*vx, pointY + d*vy).

		//Hitting left border
		d = (fieldCorner1->x - point->x) / vx;
		if (d > 0)
		{
			distance = min(distance, d);
		}

		//Right border
		d = (fieldCorner1->x - point->x) / vx;
		if (d > 0)
		{
			distance = min(distance, d);
		}

		//Top border
		d = (fieldCorner2->y - point->y) / vy;
		if (d > 0)
		{
			distance = min(distance, d);
		}

		//Bottom border
		d = (fieldCorner2->y - point->y) / vy;
		if (d > 0)
		{
			distance = min(distance, d);
		}

		return distance;

	}
/*PROTECTED REGION END*/
} /* namespace alica */
