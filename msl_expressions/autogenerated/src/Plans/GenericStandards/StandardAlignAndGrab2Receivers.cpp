using namespace std;
#include "Plans/GenericStandards/StandardAlignAndGrab2Receivers.h"

/*PROTECTED REGION ID(inccpp1462368682104) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include "engine/model/EntryPoint.h"
#include "engine/RunningPlan.h"
#include "engine/Assignment.h"
#include "engine/model/Plan.h"
#include <MSLWorldModel.h>
#include <container/CNPoint2D.h>
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1462368682104) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	StandardAlignAndGrab2Receivers::StandardAlignAndGrab2Receivers() :
			DomainBehaviour("StandardAlignAndGrab2Receivers")
	{
		/*PROTECTED REGION ID(con1462368682104) ENABLED START*/ //Add additional options here
		this->maxPassDist = 0;
		this->ratio = 0;
		this->minPassDist = 0;
		this->closerFactor = 0;
		this->ballRadius = 0;
		this->minOppDist = 0;
		this->distToFieldBorder = 0;
		this->sc = nullptr;
		this->minCloserOffset = 0;
		this->passCorridorWidth = 0;
		this->freeOppAngle = 0;
		this->maxTurnAngle = 0;
		this->canPass = false;
		this->startTime = 0;
		this->tol = 0;
		this->minTol = 0;
		this->oldAngleErr = 0;
		this->angleIntErr = 0;
		this->trans = 0;
		this->haveBallCounter = 0;
		/*PROTECTED REGION END*/
	}
	StandardAlignAndGrab2Receivers::~StandardAlignAndGrab2Receivers()
	{
		/*PROTECTED REGION ID(dcon1462368682104) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void StandardAlignAndGrab2Receivers::run(void* msg)
	{
		/*PROTECTED REGION ID(run1462368682104) ENABLED START*/ //Add additional options here
		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData->getOwnPositionVision();
		shared_ptr<geometry::CNPoint2D> egoBallPos = wm->ball->getEgoBallPosition();
		// return if necessary information is missing
		if (ownPos == nullptr || egoBallPos == nullptr)
		{
			return;
		}

		this->canPass = false;
		shared_ptr<geometry::CNPosition> alloTarget = nullptr;
		shared_ptr<geometry::CNPoint2D> alloBall = egoBallPos->egoToAllo(*ownPos);

		int recId = -1;
		int aRecId = -1;
		EntryPoint* ep = getParentEntryPoint(teamMateTaskName1);
		if (ep != nullptr)
		{
			this->recPos1 = this->getTeammatesPosition(ep);
			// TODO recId herausfinden
		}

		EntryPoint* ep2 = getParentEntryPoint(teamMateTaskName2);
		if (ep2 != nullptr)
		{
			this->recPos1 = this->getTeammatesPosition(ep);
			// TODO aRecId herausfinden
		}

		if (recPos1 == nullptr && recPos2 == nullptr)
		{
			return;
		}

		if (recId != -1)
		{
			auto vNet = this->wm->pathPlanner->getCurrentVoronoiNet();
			shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> vertices = vNet->getTeamMateVerticesCNPoint2D(recId);

			if (vertices == nullptr || this->recPos1 == nullptr)
			{
				cout << "SAAG2R: (vertices or this->recPos1) == nullptr!" << endl;
				return;
			}

			for (auto passPoint : *vertices)
			{
				// make the passPoints closer to the receiver

				shared_ptr<geometry::CNPoint2D> rcv2PassPoint = passPoint - this->recPos1;
				double rcv2PassPointDist = rcv2PassPoint->length() * closerFactor;
				if (rcv2PassPointDist > minCloserOffset)
				{
					rcv2PassPointDist = rcv2PassPoint->length() - minCloserOffset;
				}
				rcv2PassPointDist = max(rcv2PassPointDist, 0.0);
				passPoint = this->recPos1 + rcv2PassPoint->normalize() * rcv2PassPointDist;

				// pass point must be inside the field with distance to side line of "distToFieldBorder" metre
				if (!wm->field->isInsideField(passPoint, distToFieldBorder)
					|| alloBall->distanceTo(passPoint) > maxPassDist // max dist to pass point
					|| alloBall->distanceTo(passPoint) < minPassDist // min dist to pass point
					|| wm->field->isInsidePenalty(passPoint) // pass point not in any penalty area
					)
				{
					continue;
				}

				// distance from pass point to opponent is too small
				auto obs = vNet->getOpponentPositions();
				bool opponentTooClose = false;
				for (int i = 0; i < obs->size(); i++)
				{
					if (obs->at(i)->distanceTo(passPoint) < minOppDist)
					{
						opponentTooClose = true;
						break;
					}
				}
				if (opponentTooClose)
				{
					continue;
				}

				// angle between kicker and pass point is too big
				if (geometry::absDeltaAngle(
						ownPos->theta + M_PI,
						(passPoint - ownPos)->angleTo())
						> maxTurnAngle)
				{
					continue;
				}

				// some calculation to check whether any opponent is inside the pass vector triangle
				shared_ptr<geometry::CNPoint2D> ball2PassPoint = passPoint - alloBall;
				double passLength = ball2PassPoint->length();
				shared_ptr<geometry::CNPoint2D> ball2PassPointOrth = make_shared<geometry::CNPoint2D>(
						-ball2PassPoint->y, ball2PassPoint->x)->normalize() * ratio * passLength;
				shared_ptr<geometry::CNPoint2D> left = passPoint + ball2PassPointOrth;
				shared_ptr<geometry::CNPoint2D> right = passPoint - ball2PassPointOrth;
				if (!outsideTriangle(alloBall, right, left, ballRadius, vNet->getObstaclePositions())
					&& !outsideCorridore(alloBall, passPoint, this->passCorridorWidth, vNet->getObstaclePositions()))
				{
					continue;
				}

				// no opponent was in dangerous distance to our pass vector, now check our teammates with other parameters
				auto matePoses = wm->robots->teammates.getTeammatesAlloClustered();
				if (matePoses != nullptr
						&& !outsideCorridoreTeammates(alloBall, passPoint, this->ballRadius * 4, matePoses))
				{
					continue;
				}
				else
				{
					// we found a pass point, but it is not saved anywhere
					canPass = true;
				}
			}
		}

		if (canPass)
		{
			alloTarget = recPos1;
		}
		else
		{
			alloTarget = recPos2;
		}

		msl_actuator_msgs::MotionControl mc;
		if (egoBallPos->length() > 900)
		{
			// Drive close to the ball, until dist < 900
			mc = msl::RobotMovement::moveToPointCarefully(egoBallPos, egoBallPos, 0, nullptr);
			cout << "SAAG2R: egoBallPos->length() > 900 ROT: \t" << mc.motion.rotation << endl;
			send(mc);
			return;
		}

		bool haveBall = wm->ball->haveBall();
		if (!haveBall)
		{
			this->haveBallCounter = 0;
		}

		if (egoBallPos->length() > 450)
		{
			// Drive closer to the ball, but don't rotate
			mc = msl::RobotMovement::moveToPointCarefully(egoBallPos, egoBallPos, 0, nullptr);
			mc.motion.rotation = 0;
			mc.motion.translation = min(600.0, egoBallPos->length() / 1.66);
			cout << "SAAG2R: egoBallPos->length() > 450 ROT: \t" << mc.motion.rotation << endl;
			send(mc);
			return;
		}

		double radian = egoBallPos->length();
		double rot = this->trans / radian;

		shared_ptr<geometry::CNPosition> matePos;

		shared_ptr<geometry::CNPoint2D> egoMatePos = alloTarget->alloToEgo(*ownPos);

		shared_ptr<geometry::CNPoint2D> direction = nullptr;

		double dangle = geometry::deltaAngle(wm->kicker->kickerAngle, egoMatePos->angleTo());

		double cross = egoMatePos->x * egoBallPos->y - egoMatePos->y * egoBallPos->x;
		double fac = -(cross > 0 ? 1 : -1);
		if (fabs(dangle) < 12.0 * M_PI / 180.0)
		{
			direction = egoBallPos->rotate(-fac * M_PI / 2.0)->normalize() * this->trans * 0.66;
		}
		else
		{
			direction = egoBallPos->rotate(-fac * M_PI / 2.0)->normalize() * this->trans;
		}

		double balldangle = geometry::deltaAngle(wm->kicker->kickerAngle, egoBallPos->angleTo());
		if (egoBallPos->length() > 350 && fabs(dangle) > 35.0 * M_PI / 180.0)
		{
			mc.motion.angle = direction->angleTo();
			mc.motion.translation = direction->length() * 1.6;
			mc.motion.rotation = fac * rot * 1.6;
			cout << "SAAG2R: egoBallPos->length() > 350 && fabs(dangle) > 35.0 * M_PI / 180.0 ROT: \t"
					<< mc.motion.rotation << endl;
			send(mc);
			return;
		}

		if (!haveBall)
		{
			if (fabs(balldangle) > 20.0 * M_PI / 180.0)
			{
				mc.motion.rotation = (balldangle > 0 ? 1 : -1) * 0.8;
				mc.motion.angle = M_PI;
				mc.motion.translation = 100;
				cout << "SAAG2R: fabs(balldangle) > 20.0 * M_PI / 180.0 ROT: \t" << mc.motion.rotation << endl;
				send(mc);
				return;
			}
			else
			{
				mc.motion.rotation = balldangle * 0.5;
				mc.motion.angle = egoBallPos->angleTo();
				mc.motion.translation = egoBallPos->length() * 1.5;
				cout << "SAAG2R: fabs(balldangle) > 20.0 * M_PI / 180.0 else ROT: \t" << mc.motion.rotation << endl;
				send(mc);
				return;
			}
		}
		else
		{
			haveBallCounter++;
			double runningTimeMS = (double)((this->wm->getTime() - this->startTime) / 1000000ul);
			if (runningTimeMS > 9000)
			{
				mc.motion.angle = M_PI;
				mc.motion.rotation = 0.0;
				mc.motion.translation = 100.0;
				//                cout << "SAAG2R: haveBall" << endl;
				this->setSuccess(true);
			}
			else if (haveBallCounter > 6
					&& ((runningTimeMS <= 4000.0 && fabs(dangle) < this->minTol)
							|| fabs(dangle)
									< this->minTol
											+ max(0.0, (this->tol - this->minTol) / (5000.0 / (runningTimeMS - 4000.0)))))
			{
				mc.motion.angle = M_PI;
				mc.motion.rotation = 0.0;
				mc.motion.translation = 100.0;
				//                cout << "SAAG2R: haveBall else if" << endl;
				this->setSuccess(true);
			}
		}
		cout << "SAAG2R: last mc ROT: \t" << mc.motion.rotation << endl;
		send(mc);

		/*PROTECTED REGION END*/
	}
	void StandardAlignAndGrab2Receivers::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1462368682104) ENABLED START*/ //Add additional options here
		this->haveBallCounter = 0;
		this->angleIntErr = 0;
		this->oldAngleErr = 0;
		this->haveBallCounter = 0;
		this->startTime = wm->getTime();
		auto sc = supplementary::SystemConfig::getInstance();
		this->minCloserOffset = (*this->sc)["Behaviour"]->get<double>("Pass", "MinCloserOffset", NULL);
		this->closerFactor = (*this->sc)["Behaviour"]->get<double>("Pass", "CloserFactor",
		NULL);
		this->ballRadius = (*this->sc)["Rules"]->get<double>("Rules.BallRadius",
		NULL);
		this->freeOppAngle = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "freeOppAngle",
		NULL) / 2;
		this->ratio = tan(this->freeOppAngle);
		this->passCorridorWidth = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "passCorridorWidth", NULL);
		this->maxTurnAngle = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "maxTurnAngle",
		NULL);
		this->minOppDist = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "minOppDist",
		NULL);
		this->minPassDist = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "minPassDist",
		NULL);
		this->maxPassDist = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "maxPassDist",
		NULL);
		this->distToFieldBorder = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "distToFieldBorder", NULL);
		string tmp;
		bool success = true;
		try
		{
			success &= getParameter("TeamMateTaskName1", tmp);
			if (success)
			{
				teamMateTaskName1 = tmp;
			}

			success &= getParameter("TeamMateTaskName", tmp);
			if (success)
			{
				teamMateTaskName2 = tmp;
			}

		}
		catch (exception& e)
		{
			cerr << "Could not cast the parameter properly" << endl;
		}
		if (!success)
		{
			cerr << "SAAG2R: Parameter does not exist" << endl;
		}

		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1462368682104) ENABLED START*/ //Add additional methods here
	bool StandardAlignAndGrab2Receivers::outsideCorridore(shared_ptr<geometry::CNPoint2D> ball,
															shared_ptr<geometry::CNPoint2D> passPoint,
															double passCorridorWidth,
															shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points)
	{
		for (int i = 0; i < points->size(); i++)
		{
			if (geometry::distancePointToLineSegment(points->at(i)->x, points->at(i)->y, ball, passPoint)
			< passCorridorWidth)
			{
				return false;
			}
		}
		return true;
	}

	bool StandardAlignAndGrab2Receivers::outsideCorridoreTeammates(shared_ptr<geometry::CNPoint2D> ball,
																	shared_ptr<geometry::CNPoint2D> passPoint,
																	double passCorridorWidth,
																	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points)
	{
		for (int i = 0; i < points->size(); i++)
		{
			if (geometry::distancePointToLineSegment(points->at(i)->x, points->at(i)->y, ball, passPoint)
			< passCorridorWidth && ball->distanceTo(points->at(i)) < ball->distanceTo(passPoint) - 100)
			{
				return false;
			}
		}
		return true;
	}

	bool StandardAlignAndGrab2Receivers::outsideTriangle(shared_ptr<geometry::CNPoint2D> a,
															shared_ptr<geometry::CNPoint2D> b,
															shared_ptr<geometry::CNPoint2D> c, double tolerance,
															shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points)
	{
		shared_ptr<geometry::CNPoint2D> a2b = b - a;
		shared_ptr<geometry::CNPoint2D> b2c = c - b;
		shared_ptr<geometry::CNPoint2D> c2a = a - c;
		shared_ptr<geometry::CNPoint2D> a2p;
		shared_ptr<geometry::CNPoint2D> b2p;
		shared_ptr<geometry::CNPoint2D> c2p;
		shared_ptr<geometry::CNPoint2D> p;
		for (int i = 0; i < points->size(); i++)
		{
			p = points->at(i);
			a2p = p - a;
			b2p = p - b;
			c2p = p - c;

			if ((a2p->x * a2b->y - a2p->y * a2b->x) / a2p->normalize()->length()< tolerance
				&& (b2p->x * b2c->y - b2p->y * b2c->x) / b2p->normalize()->length() < tolerance
				&& (c2p->x * c2a->y - c2p->y * c2a->x) / c2p->normalize()->length() < tolerance)
				{
					return false;
				}

			}
			return true;
	}
	/*PROTECTED REGION END*/
}
/* namespace alica */
