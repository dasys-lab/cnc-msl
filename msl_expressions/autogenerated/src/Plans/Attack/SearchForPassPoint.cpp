using namespace std;
#include "Plans/Attack/SearchForPassPoint.h"

/*PROTECTED REGION ID(inccpp1436269017402) ENABLED START*/ //Add additional includes here
#include <GeometryCalculator.h>
#include "pathplanner/VoronoiNet.h"
#include "pathplanner/PathProxy.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1436269017402) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    SearchForPassPoint::SearchForPassPoint() :
            DomainBehaviour("SearchForPassPoint")
    {
        /*PROTECTED REGION ID(con1436269017402) ENABLED START*/ //Add additional options here
        this->pathProxy = msl::PathProxy::getInstance();
        /*PROTECTED REGION END*/
    }
    SearchForPassPoint::~SearchForPassPoint()
    {
        /*PROTECTED REGION ID(dcon1436269017402) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void SearchForPassPoint::run(void* msg)
    {
        /*PROTECTED REGION ID(run1436269017402) ENABLED START*/ //Add additional options here
        if (eps.size() <= 0)
        {
            cout << "S4PP: All EPs is null" << endl;
            return;
        }

        shared_ptr < geometry::CNPoint2D > alloBall = this->wm->ball.getAlloBallPosition();
        if (alloBall == nullptr)
        {
            cout << "S4PP: Ball is null" << endl;
            return;
        }

        shared_ptr < geometry::CNPosition > alloPos = this->wm->rawSensorData.getOwnPositionVision();
        if (alloPos == nullptr)
        {
            cout << "S4PP: OwnPos is null" << endl;
            return;
        }

        // ensures, that we have the ball and are not in melee with some opp.
        if (!true && this->wm->game.getGameState() != msl::GameState::OwnBallPossession)
        {
            cout << "S4PP: Gamestate is not Attack" << endl;
            return;
        }

        // the only teammate in the corresponding task/ entrypoint
        teamMateIds.clear();
        for (EntryPoint* ep : eps)
        {
            auto teammates = robotsInEntryPointOfHigherPlan(ep);
            for (int mateId : *teammates)
            {
                this->teamMateIds.push_back(mateId);
                break;
            }
        }
        if (teamMateIds.size() <= 0)
        {
            cout << "S4PP: Somethine Strange is going on with RobotIDs and Entrypoints" << endl;
            return;
        }

        shared_ptr < msl::VoronoiNet > vNet = this->wm->pathPlanner.getCurrentVoronoiNet();
        if (vNet == nullptr)
        {
            cout << "vnet null " << endl;
            return;
        }
        try
        {
            shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> sites = make_shared<
                    vector<shared_ptr<geometry::CNPoint2D>>>();
            for (int teamMateId : this->teamMateIds)
            {

                shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> vertices = vNet->getTeamMateVertices(teamMateId);
                shared_ptr < geometry::CNPosition > teamMatePos = wm->robots.getTeamMatePosition(teamMateId);
                for (int i = 0; i < vertices->size(); i++)
                {
                    // make the passpoints closer to the receiver
                    shared_ptr < geometry::CNPoint2D > passPoint = vertices->at(i);
                    shared_ptr < geometry::CNPoint2D > receiver = make_shared < geometry::CNPoint2D
                    > (teamMatePos->x, teamMatePos->y);
                    shared_ptr < geometry::CNPoint2D > rcv2PassPoint = passPoint - receiver;
                    double rcv2PassPointDist = rcv2PassPoint->length();
                    double factor = closerFactor;
                    if (factor * rcv2PassPointDist > minCloserOffset)
                    {
                        factor = factor * rcv2PassPointDist;
                    }
                    else
                    {
                        factor = rcv2PassPointDist - minCloserOffset;
                    }
                    factor = max(factor, 0.0);
                    passPoint = receiver + rcv2PassPoint->normalize() * factor;

                    if (ff->isInsideField(passPoint, distToFieldBorder) // pass point must be inside the field with distance to side line of 1.5 metre
                    && !ff->isInsidePenalty(passPoint, 0.0) && alloBall->distanceTo(passPoint) < maxPassDist// max dist to pass point
                    && alloBall->distanceTo(passPoint) > minPassDist// min dist to pass point
                    )
                    {

//						// min dist to opponent
                        auto obs = vNet->getOpponentPositions();
                        bool opponentTooClose = false;
                        for (int i = 0; i < obs->size(); i++)
                        {
                            if (obs->at(i).first->distanceTo(passPoint) < minOppDist)
                            {
                                opponentTooClose = true;
                                break;
                            }
                        }
                        if (opponentTooClose)
                        {
                            continue;
                        }
//						if ((vertices->at(i).tri.p[0].ident == -1 && vertices->at(i).tri.p[0].DistanceTo(passPoint) < minOppDist)
//								||(vertices->at(i).tri.p[1].ident == -1 && vertices->at(i).tri.p[1].DistanceTo(passPoint) < minOppDist)
//								||(vertices->at(i).tri.p[2].ident == -1 && vertices->at(i).tri.p[2].DistanceTo(passPoint) < minOppDist))
//						{
//							continue;
//						}

                        //small angle to turn to pass point
                        if (geometry::GeometryCalculator::absDeltaAngle(
                                alloPos->theta + M_PI,
                                (passPoint - make_shared < geometry::CNPoint2D > (alloPos->x, alloPos->y))->angleTo())
                        > maxTurnAngle)
                        {
                            continue;
                        }

                        // some calculation to check whether any opponent is inside the pass vector triangle
                        shared_ptr < geometry::CNPoint2D > ball2PassPoint = passPoint - alloBall;
                        double passLength = ball2PassPoint->length();
                        shared_ptr < geometry::CNPoint2D > ball2PassPointOrth = make_shared < geometry::CNPoint2D
                        > (-ball2PassPoint->y, ball2PassPoint->x)->normalize() * ratio * passLength;
                        shared_ptr < geometry::CNPoint2D > left = passPoint + ball2PassPointOrth;
                        shared_ptr < geometry::CNPoint2D > right = passPoint - ball2PassPointOrth;
                        if (!outsideTriangle(alloBall, right, left, ballRadius, vNet->getObstaclePositions())
                        && !outsideCorridore(alloBall, passPoint, this->passCorridorWidth,
                                vNet->getObstaclePositions()))
                        {
                            continue;
                        }

                        // no opponent was in dangerous distance to our pass vector, now check our teammates with other parameters
                        if (!outsideCorridoreTeammates(alloBall, passPoint, this->ballRadius * 4,
                                vNet->getTeamMatePositions()))
                        {
                            continue;
                        }
                        else
                        {
                            sites->push_back(passPoint);
                            pathProxy->sendVoronoiNetMsg(sites, vNet);
                            this->success = true;
//                            return;
                        }
                    }
                }
            }
        }
        catch (exception& e)
        {
            throw e;
        }

        /*PROTECTED REGION END*/
    }
    void SearchForPassPoint::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1436269017402) ENABLED START*/ //Add additional options here
        teamMatePlanName.clear();
        teamMateTaskName.clear();
        sc = supplementary::SystemConfig::getInstance();
        this->minCloserOffset = (*this->sc)["Behaviour"]->get<double>("Pass", "MinCloserOffset", NULL);
        this->closerFactor = (*this->sc)["Behaviour"]->get<double>("Pass", "CloserFactor", NULL);
        string tmp;
        string tmp2;
        bool success = true;
        success &= getParameter("DistToFieldBorder", tmp);
        try
        {
            if (success)
            {
                this->distToFieldBorder = stod(tmp);
            }

            success &= getParameter("FreeOppAngle", tmp);
            if (success)
            {
                this->freeOppAngle = stod(tmp) / 2;
                this->ratio = tan(freeOppAngle);
            }
            success &= getParameter("MaxPassDist", tmp);
            if (success)
            {
                this->maxPassDist = stod(tmp);
            }
            success &= getParameter("MaxTurnAngle", tmp);
            if (success)
            {
                this->maxTurnAngle = stod(tmp);
            }
            success &= getParameter("MinOppDist", tmp);
            if (success)
            {
                this->minOppDist = stod(tmp);
            }
            success &= getParameter("MinPassDist", tmp);
            if (success)
            {
                this->minPassDist = stod(tmp);
            }
            success &= getParameter("PassCorridorWidth", tmp);
            if (success)
            {
                this->passCorridorWidth = stod(tmp);
            }
            int iter = 0;
            stringstream ss;
            stringstream ss2;
            while (true)
            {
                ss << "TeamMateTaskName" << iter;
                ss2 << "TeamMatePlanName" << iter;
                if (getParameter(ss.str(), tmp) && getParameter(ss2.str(), tmp2))
                {
                    teamMateTaskName.push_back(tmp);
                    teamMatePlanName.push_back(tmp2);
                }
                else
                {
                    break;
                }
                ss.str("");
                ss2.str("");
                iter++;
            }
            eps.clear();
            if (success && teamMatePlanName.size() != 0 && teamMateTaskName.size() != 0)
            {
                for (int i = 0; i < teamMatePlanName.size(); i++)
                {
                    EntryPoint* ep = getHigherEntryPoint(teamMatePlanName[i], teamMateTaskName[i]);
                    if (ep != nullptr)
                    {
                        eps.push_back(ep);
                    }
                }
            }

        }
        catch (exception& e)
        {
            cerr << "Could not cast the parameter properly" << endl;
        }
        if (!success)
        {
            cerr << "Parameter does not exist" << endl;
        }
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1436269017402) ENABLED START*/ //Add additional methods here
    bool SearchForPassPoint::outsideCorridore(shared_ptr<geometry::CNPoint2D> ball,
                                              shared_ptr<geometry::CNPoint2D> passPoint, double passCorridorWidth,
                                              shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> points)
    {
        for (int i = 0; i < points->size(); i++)
        {
            if (geometry::GeometryCalculator::distancePointToLineSegment(points->at(i).first->x, points->at(i).first->y, ball, passPoint)
            < passCorridorWidth)
            {
                return false;
            }
        }
        return true;
    }

    bool SearchForPassPoint::outsideCorridoreTeammates(shared_ptr<geometry::CNPoint2D> ball,
                                                       shared_ptr<geometry::CNPoint2D> passPoint,
                                                       double passCorridorWidth,
                                                       shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> points)
    {
        for (int i = 0; i < points->size(); i++)
        {
            if (geometry::GeometryCalculator::distancePointToLineSegment(points->at(i).first->x, points->at(i).first->y, ball, passPoint)
            < passCorridorWidth && ball->distanceTo(points->at(i).first) < ball->distanceTo(passPoint) - 100)
            {
                return false;
            }
        }
        return true;
    }

    bool SearchForPassPoint::outsideTriangle(shared_ptr<geometry::CNPoint2D> a, shared_ptr<geometry::CNPoint2D> b,
                                             shared_ptr<geometry::CNPoint2D> c, double tolerance,
                                             shared_ptr<vector<pair<shared_ptr<geometry::CNPoint2D>, int>>> points)
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
                                                 p = points->at(i).first;
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
		} /* namespace alica */
