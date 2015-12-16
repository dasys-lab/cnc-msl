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
		wheelSpeed = -100; // TODO config?
		translation = 800;
		fieldLength = (*this->sc)["Globals"]->get<double>("FootballField.FieldLength", NULL);
		direction = 0;
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
		shared_ptr<geometry::CNPosition> me = wm->rawSensorData.getOwnPositionVision();
		shared_ptr<geometry::CNPoint2D> ownPoint = make_shared<geometry::CNPoint2D>(me->x, me->y);
		shared_ptr<geometry::CNPoint2D> egoBallPos = wm->ball.getEgoBallPosition();

		if (me == nullptr || egoBallPos == nullptr)
		{
			return;
		}

		msl_actuator_msgs::BallHandleCmd bhc;
		bhc.leftMotor = wheelSpeed;
		bhc.rightMotor = wheelSpeed;



		send(bhc);

		shared_ptr<geometry::CNPoint2D> ownGoalPos = make_shared<geometry::CNPoint2D>(-fieldLength / 2, 0.0);

		if (direction == 0)
		{
			if (ownGoalPos != nullptr && me->distanceTo(ownGoalPos) < fieldLength / 3)
			{
				//own goal is close, get the ball away

				if(checkSide(*egoBallPos,*ownGoalPos)) {
					direction = -1;
				} else {
					direction = 1;
				}
			}
			else
			{

				//own goal is far away, look for nearby friends
				shared_ptr<geometry::CNPoint2D> friendly = nullptr;


				for (auto pos = wm->robots.getPositionsOfTeamMates()->begin();
						pos != wm->robots.getPositionsOfTeamMates()->end(); pos++)
				{
					//iterator richtig? pos null?

					shared_ptr<geometry::CNPoint2D> friendlyPos = make_shared<geometry::CNPoint2D>(
							pos->get()->second->x, pos->get()->second->y);


					//TODO sind die 2000 notwendig? und in konstante packen
					if (friendlyPos->distanceTo(ownPoint) < 2000)
					{
						if (friendly == nullptr || friendly->distanceTo(ownPoint) < friendlyPos->distanceTo(ownPoint))
						{
							friendly = friendlyPos;
						}
					}

					if (friendly != nullptr)
					{
						//found one, try to get the ball to him
						if(checkSide(*egoBallPos,*friendly)) {
							direction = 1;
						} else {
							direction = -1;
						}
					}

					// test
					 // !test
					else if (me != nullptr)
					{
						//found none, looking for free space

						// artificial obs ignorieren, kann man das so machen?
//						auto currentObstacles = wm->robots.getObstacles(0);
//						auto artificialObs = wm->pathPlanner.getArtificialObstacles();
//						for (auto it1 = currentObstacles->begin(); it1 != currentObstacles->end(); it1++) {
//							for(auto it2 = artificialObs->begin(); it2 != artificialObs->end(); it2++) {
//								//TODO toleranz
//
//								geometry::CNPoint2D rectPointA = geometry::CNPoint2D(it2->get()->x + it2->get()->normalize()* )
//								if(it1->x == it2->get()->x && it1->y == it2->get()->y) {
//									currentObstacles->erase(it1);
//								}
//							}
//						}


						//iterate over tracked opponents
						//for every opp, check if opp is closer than 2m to the ball and angle difference is smaller than 270deg (but why?)
						//if yes, check if opp is left or right of ball -> set direction over posFree and negFree bool?!
						//if all occupied, try closest field border (never happens)
					}
				}

			}
		}

		msl_actuator_msgs::MotionControl mc;
		mc.motion.angle = egoBallPos->angleTo() + direction*M_PI;
		mc.motion.rotation = direction * M_PI/2.0;
		mc.motion.translation = translation;

		send(mc);

		/*PROTECTED REGION END*/
	}
	void Duel::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1450178699265) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1450178699265) ENABLED START*/ //Add additional methods here

	// returns true if pointToCheck is left of lineVector(look along the lineVector)
	// uses cross product of 2 vectors. 0: colinear, <0: point left of vec, >0: point right of vec
	bool Duel::checkSide(geometry::CNPoint2D lineVector, geometry::CNPoint2D pointToCheck)
	{
		double cross = pointToCheck.x * lineVector.y - pointToCheck.y * lineVector.x;
		return (cross < 0);
	}
/*PROTECTED REGION END*/
} /* namespace alica */
