using namespace std;
#include "Plans/Attack/SearchForPassPoint.h"

/*PROTECTED REGION ID(inccpp1436269017402) ENABLED START*/ //Add additional includes here
#include <GeometryCalculator.h>
#include "pathplanner/VoronoiNet.h"
#include "pathplanner/PathProxy.h"
#include "msl_helper_msgs/PassMsg.h"
#include "msl_helper_msgs/DebugMsg.h"
#include "msl_helper_msgs/DebugPoint.h"
#include <Robots.h>
#include <RawSensorData.h>
#include <Ball.h>
#include <pathplanner/PathPlanner.h>
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
        maxPassDist = 0;
        ratio = 0;
        minPassDist = 0;
        closerFactor = 0;
        closerFactor2 = 0;
        ballRadius = 0;
        minOppDist = 0;
        distToFieldBorder = 0;
        sc = nullptr;
        minCloserOffset = 0;
        passCorridorWidth = 0;
        freeOppAngle = 0;
        maxTurnAngle = 0;
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
//            cout << "S4PP: All EPs is null" << endl;
            return;
        }

        alloBall = this->wm->ball->getAlloBallPosition();
        if (alloBall == nullptr)
        {
            cout << "S4PP: Ball is null" << endl;
            return;
        }

        alloPos = this->wm->rawSensorData->getOwnPositionVision();
        if (alloPos == nullptr)
        {
            cout << "S4PP: OwnPos is null" << endl;
            return;
        }

        // the only teammate in the corresponding task/ entrypoint
        this->teamMateIds.clear();
        for (EntryPoint* ep : eps)
        {
            auto teammates = robotsInEntryPointOfHigherPlan(ep);

            if (teammates == nullptr)
            {
                cout << "S4PP: No Teammate for entry point " << ep->toString() << endl;
                return;
            }

            for (int mateId : *teammates)
            {
                this->teamMateIds.push_back(mateId);
                break;
            }
        }
        if (this->teamMateIds.size() <= 0)
        {
            cout << "S4PP: Something Strange is going on with RobotIDs and Entrypoints" << endl;
            return;
        }

        shared_ptr < msl::VoronoiNet > vNet = this->wm->pathPlanner->getCurrentVoronoiNet();
        if (vNet == nullptr)
        {
            cout << "vnet null " << endl;
            return;
        }

        matePoses = wm->robots->teammates.getTeammatesAlloClustered();
        if (matePoses == nullptr)
        {
            cout << "matePoses == nullptr" << endl;
            return;
        }
        for (auto i = matePoses->begin(); i != matePoses->end(); i++)
        {
            if ((*i)->distanceTo(alloPos) < 100)
            {
                matePoses->erase(i);
                break;
            }
        }

        try
        {
#ifdef DBM_DEBUG
            dbm = make_shared<msl_helper_msgs::DebugMsg>();
            dbm->topic = "Pass";
#endif
            for (int teamMateId : this->teamMateIds)
            {

                shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> vertices = vNet->getTeamMateVerticesCNPoint2D(
                        teamMateId);
                shared_ptr < geometry::CNPosition > teamMatePos = wm->robots->teammates.getTeamMatePosition(teamMateId);

                if (vertices == nullptr || teamMatePos == nullptr)
                    continue;

                for (int i = 0; i < vertices->size(); i++)
                {
                    // make the passpoints closer to the receiver
                    shared_ptr < geometry::CNPoint2D > passPoint = vertices->at(i);

                    shared_ptr < geometry::CNPoint2D > receiver = make_shared < geometry::CNPoint2D
                            > (teamMatePos->x, teamMatePos->y);

                    if (passPossible(this->closerFactor, passPoint, receiver, vNet))
                    {
                        this->setSuccess(true);
                    }
                    if (passPossible(this->closerFactor2, passPoint, receiver, vNet))
                    {
                        this->setSuccess(true);
                    }

                }
                if (passPossible(0.0, teamMatePos->getPoint(), teamMatePos->getPoint(), vNet))
                {
                    this->setSuccess(true);
                }
            }
#ifdef DBM_DEBUG
            send(*dbm);
            dbm = nullptr;
#endif
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
        this->closerFactor2 = (*this->sc)["Behaviour"]->get<double>("Pass", "CloserFactor2", NULL);
        this->ballRadius = (*this->sc)["Rules"]->get<double>("Rules.BallRadius", NULL);
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
            cerr << "S4PP: Could not cast the parameter properly" << endl;
        }
        if (!success)
        {
            cerr << "S4PP: Parameter does not exist" << endl;
        }
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1436269017402) ENABLED START*/ //Add additional methods here
    bool SearchForPassPoint::passPossible(double cf, shared_ptr<geometry::CNPoint2D> passPoint,
                                          shared_ptr<geometry::CNPoint2D> receiver, shared_ptr<msl::VoronoiNet> vNet)
    {
        shared_ptr < geometry::CNPoint2D > rcv2PassPoint = passPoint - receiver;
        double rcv2PassPointDist = rcv2PassPoint->length();
        double factor = cf;
        if (factor * rcv2PassPointDist < minCloserOffset)
        {
            factor = factor * rcv2PassPointDist;
        }
        else
        {
            factor = rcv2PassPointDist - minCloserOffset;
        }
        factor = max(factor, 0.0);
        if (rcv2PassPoint->x != 0 && rcv2PassPoint->y != 0)
        {
            passPoint = receiver + rcv2PassPoint->normalize() * factor;
        }
        else
        {
            passPoint = receiver;
        }

#ifdef DBM_DEBUG
        msl_helper_msgs::DebugPoint dbp;
        dbp.point.x = passPoint->x;
        dbp.point.y = passPoint->y;
        dbp.radius = 0.3;
        dbm->points.push_back(dbp);
#endif
        if (wm->field->isInsideField(passPoint, distToFieldBorder) // pass point must be inside the field with distance to side line of 1.5 metre
        && !wm->field->isInsidePenalty(passPoint, 0.0) && alloBall->distanceTo(passPoint) < maxPassDist // max dist to pass point
        && alloBall->distanceTo(passPoint) > minPassDist // min dist to pass point
                )
        {

            //						// min dist to opponent
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
#ifdef DBM_DEBUG
                dbm->points.at(dbm->points.size() - 1).red = 0.2 * 255.0;
                dbm->points.at(dbm->points.size() - 1).green = 0.2 * 255.0;
                dbm->points.at(dbm->points.size() - 1).blue = 0.2 * 255.0;
#endif
                return false;
            }
            //small angle to turn to pass point
            if (geometry::absDeltaAngle(
                    alloPos->theta + M_PI,
                    (passPoint - make_shared < geometry::CNPoint2D > (alloPos->x, alloPos->y))->angleTo())
                    > maxTurnAngle)
            {
#ifdef DBM_DEBUG
                dbm->points.at(dbm->points.size() - 1).red = 0.0 * 255.0;
                dbm->points.at(dbm->points.size() - 1).green = 0.4 * 255.0;
                dbm->points.at(dbm->points.size() - 1).blue = 0.0 * 255.0;
#endif
                return false;
            }

            // some calculation to check whether any opponent is inside the pass vector triangle
            shared_ptr < geometry::CNPoint2D > ball2PassPoint = passPoint - alloBall;
            double passLength = ball2PassPoint->length();
            shared_ptr < geometry::CNPoint2D > ball2PassPointOrth = make_shared < geometry::CNPoint2D
                    > (-ball2PassPoint->y, ball2PassPoint->x)->normalize() * ratio * passLength;
            shared_ptr < geometry::CNPoint2D > left = passPoint + ball2PassPointOrth;
            shared_ptr < geometry::CNPoint2D > right = passPoint - ball2PassPointOrth;
            auto obsPositions = vNet->getObstaclePositions();
            if (!geometry::outsideTriangle(alloBall, right, left, ballRadius, obsPositions)
                    && !outsideCorridore(alloBall, passPoint, this->passCorridorWidth, obsPositions))
            {
#ifdef DBM_DEBUG
                dbm->points.at(dbm->points.size() - 1).red = 0.6 * 255.0;
                dbm->points.at(dbm->points.size() - 1).green = 0.0 * 255.0;
                dbm->points.at(dbm->points.size() - 1).blue = 0.0 * 255.0;
#endif
                return false;
            }

            // no opponent was in dangerous distance to our pass vector, now check our teammates with other parameters
            if (!outsideCorridoreTeammates(alloBall, passPoint, this->ballRadius * 4, matePoses))
            {
#ifdef DBM_DEBUG
                dbm->points.at(dbm->points.size() - 1).red = 0.0 * 255.0;
                dbm->points.at(dbm->points.size() - 1).green = 0.0 * 255.0;
                dbm->points.at(dbm->points.size() - 1).blue = 0.8 * 255.0;
#endif
                return false;
            }
            else
            {
#ifdef DBM_DEBUG
                dbm->points.at(dbm->points.size() - 1).red = 255;
                dbm->points.at(dbm->points.size() - 1).green = 255;
                dbm->points.at(dbm->points.size() - 1).blue = 255;
#endif
                return true;
            }
            return false;
        }
        return false;
    }

    bool SearchForPassPoint::outsideCorridore(shared_ptr<geometry::CNPoint2D> ball,
                                              shared_ptr<geometry::CNPoint2D> passPoint, double passCorridorWidth,
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

    bool SearchForPassPoint::outsideCorridoreTeammates(shared_ptr<geometry::CNPoint2D> ball,
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
	/*PROTECTED REGION END*/			
		} /* namespace alica */
