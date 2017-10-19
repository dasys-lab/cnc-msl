using namespace std;
#include "Plans/Attack/AlignAndPassRapid.h"

/*PROTECTED REGION ID(inccpp1436269063295) ENABLED START*/ //Add additional includes here
#include "msl_helper_msgs/PassMsg.h"
#include "pathplanner/VoronoiNet.h"
#include "pathplanner/PathProxy.h"
#include "msl_helper_msgs/DebugMsg.h"
#include "msl_helper_msgs/DebugPoint.h"
#include <engine/model/EntryPoint.h>
#include <SystemConfig.h>
#include <RawSensorData.h>
#include <msl_robot/kicker/Kicker.h>
#include <Robots.h>
#include <pathplanner/PathPlanner.h>
#include <Ball.h>
#include <engine/model/EntryPoint.h>
#include <engine/Assignment.h>
#include <msl_robot/MSLRobot.h>
#include <nonstd/optional.hpp>
#include <cnc_geometry/CNPointAllo.h>

/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1436269063295) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    AlignAndPassRapid::AlignAndPassRapid() :
            DomainBehaviour("AlignAndPassRapid")
    {
        /*PROTECTED REGION ID(con1436269063295) ENABLED START*/ //Add additional options here
        this->freeOppAngle = NAN;
        this->ratio = NAN;
        this->ballRadius = NAN;
        this->passCorridorWidth = NAN;
        this->maxTurnAngle = NAN;
        this->minOppDist = NAN;
        this->minPassDist = NAN;
        this->maxPassDist = NAN;
        this->distToFieldBorder = NAN;
        this->minCloserOffset = NAN;
        this->closerFactor = NAN;
        this->closerFactor2 = NAN;
        this->arrivalTimeOffset = NAN;
        this->maxVel = 2000;
        this->pRot = 2.1;
        this->dRot = 0.0;
        this->lastRotError = 0;
        this->minRot = 0.1;
        this->maxRot = M_PI * 4;
        this->accel = 2000;
        this->sc = supplementary::SystemConfig::getInstance();
        this->pathProxy = msl::PathProxy::getInstance();
        this->alloBall = nonstd::nullopt;
        this->alloPos = nonstd::nullopt;
        this->bestPassUtility = numeric_limits<double>::min();
        this->bestTeamMateId = -1;
        this->found = false;
        this->best_point = -1;
        /*PROTECTED REGION END*/
    }
    AlignAndPassRapid::~AlignAndPassRapid()
    {
        /*PROTECTED REGION ID(dcon1436269063295) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void AlignAndPassRapid::run(void* msg)
    {
        /*PROTECTED REGION ID(run1436269063295) ENABLED START*/ //Add additional options here
        msl_actuator_msgs::MotionControl mc;
        alloPos = this->wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        if (!alloPos)
        {
//			mc = DriveHelper.DriveRandomly(500,WM);
//			Send(mc);
            cout << "AAPR: OwnPos is null" << endl;
            ;
            return;
        }

        auto egoBallPos = this->wm->ball->getPositionEgo();
        if (!egoBallPos)
        {
            cout << "AAPR: Ego Ball is null" << endl;
            return;
        }

        alloBall = this->wm->ball->getPositionAllo();
        if (!alloBall)
        {
            cout << "AAPR: Allo Ball is null" << endl;
            return;
        }

        if (eps.size() <= 0)
        {
            cout << "AAPR: All EPs is null" << endl;
            return;
        }
        // the only teammate in the corresponding task/ entrypoint
        teamMateIds.clear();
        for (EntryPoint* ep : eps)
        {
            auto teammates = robotsInEntryPointOfHigherPlan(ep);

            if (teammates == nullptr)
            {
                cout << "AAPR: No Teammate for entry point " << ep->toString() << endl;
                return;
            }

            for (int mateId : *teammates)
            {
                this->teamMateIds.push_back(mateId);
                break;
            }
        }

        if (teamMateIds.size() <= 0)
        {
            cout << "AAPR: Something Strange is going on with RobotIDs and Entrypoints" << endl;
            return;
        }

        auto vNet = this->wm->pathPlanner->getCurrentVoronoiNet();
        if (vNet == nullptr)
        {
            cout << "AAPR: VNet is null!" << endl;
            return;
        }

        matePoses = wm->robots->teammates.getTeammatesAlloClusteredBuffer().getLastValidContent();
        if (!matePoses)
        {
            cout << "matePoses == nullptr" << endl;
            return;
        }
        for (auto i = matePoses->begin(); i != matePoses->end(); i++)
        {
            if (i->distanceTo(alloPos->getPoint()) < 100)
            {
                matePoses->erase(i);
                break;
            }
        }

        bestPassUtility = numeric_limits<double>::min();
        bestTeamMateId = -1;
        found = false;

#ifdef DBM_DEBUG
        best_point = -1;
        dbm = make_shared<msl_helper_msgs::DebugMsg>();
        dbm->topic = "Pass";
#endif
        for (int teamMateId : this->teamMateIds)
        {
            auto vertices = vNet->getTeamMateVerticesCNPoint2D(teamMateId);
            auto teamMatePos = wm->robots->teammates.getTeammatePositionBuffer(teamMateId).getLastValidContent();

            if (vertices == nullptr || !teamMatePos)
                continue;
            for (int i = 0; i < vertices->size(); i++)
            {
                // make the passpoints closer to the receiver
                auto passPoint = vertices->at(i);
                auto receiver = teamMatePos->getPoint();

                findBestPassPoint(this->closerFactor, passPoint, receiver, vNet, *teamMatePos, teamMateId);
                findBestPassPoint(this->closerFactor2, passPoint, receiver, vNet, *teamMatePos, teamMateId);
                findBestPassPoint(0.0, receiver, receiver, vNet, *teamMatePos, teamMateId);

            }
        }
#ifdef DBM_DEBUG
        if (best_point >= 0)
        {
            dbm->points.at(best_point).red = 0;
            dbm->points.at(best_point).green = 0;
        }
        send(*dbm);
#endif
        if (!found)
        { // No Pass point found, so return everything
            this->setSuccess(true);
            cout << "AAPR: No valid pass point found! SuccessStatus: " << this->isSuccess() << endl;
            return;
        }
        //Turn to goal...
        auto ballVel = this->wm->ball->getVisionBallVelocityBuffer().getLastValidContent();
        geometry::CNVecEgo ballVel2;
        if (!ballVel)
        {
            ballVel2 = geometry::CNVecEgo(0, 0);
        }
        else if (ballVel->length() > 5000)
        {
            auto v = ballVel->normalize() * 5000;
            ballVel2 = geometry::CNVecEgo(v.x, v.y);
        }
        else
        {
            ballVel2 = geometry::CNVecEgo(ballVel->x, ballVel->y);
        }
        auto egoAimPoint = alloAimPoint.toEgo(*alloPos);
        double egoAimAngle = egoAimPoint.angleZ();
        double ballAngle = egoBallPos->angleZ();
        double deltaAngle = geometry::deltaAngle(ballAngle, egoAimAngle);
        if (abs(deltaAngle) < M_PI / 36)
        { // +/-5 degree
          //Kick && PassMsg
            msl_helper_msgs::PassMsg pm;
            msl_msgs::Point2dInfo pinf;
            // Distance to aim point * direction of our kicker = actual pass point destination
            double dist = egoAimPoint.length();
            auto dest = geometry::CNPointEgo(-dist, 0);
            auto alloDest = dest.toAllo(*alloPos);
            pinf.x = alloDest.x;
            pinf.y = alloDest.y;
            pm.destination = pinf;
            pinf = msl_msgs::Point2dInfo();
            pinf.x = alloPos->x;
            pinf.y = alloPos->y;
            pm.origin = pinf;
            pm.receiverID = bestTeamMateId;
            msl_actuator_msgs::KickControl km;
            km.enabled = true;
            km.kicker = 1; //(ushort)KickHelper.KickerToUseIndex(egoBallPos->angleTo());

            auto goalReceiverVec = alloDest - geometry::CNVecAllo(bestAoc.x, bestAoc.y);
            double v0 = 0;
            double distReceiver = goalReceiverVec.length();
            double estimatedTimeForReceiverToArrive = (sqrt(2 * accel * distReceiver + v0 * v0) - v0) / accel;
            //considering network delay and reaction time 1s?:
            estimatedTimeForReceiverToArrive += 1.0;
            pm.validFor = (unsigned long long)(estimatedTimeForReceiverToArrive * 1000000000.0 + 300000000.0); // this is sparta!
            if (closerFactor < 0.01)
            {
                km.power = (ushort)this->robot->kicker->getKickPowerPass(egoAimPoint.length());
            }
            else
            {
                km.power = (ushort)this->robot->kicker->getPassKickpower(
                        dist, estimatedTimeForReceiverToArrive + arrivalTimeOffset);
            }

            send(km);
            if (this->robot->kicker->lowShovelSelected)
            {
                send(pm);
            }

        }
        auto dstscan = this->wm->rawSensorData->getDistanceScanBuffer().getLastValidContent();
        if (dstscan && (*dstscan)->size() != 0)
        {
            double distBeforeBall = this->minFree(egoBallPos->angleZ(), 200, *dstscan);
            if (distBeforeBall < 250)
                this->setFailure(true);
        }
        mc = msl_actuator_msgs::MotionControl();
        mc.motion.rotation = deltaAngle * pRot + (deltaAngle - lastRotError) * dRot;
        double sign = geometry::sgn(mc.motion.rotation);
        mc.motion.rotation = sign * min(this->maxRot, max(abs(mc.motion.rotation), this->minRot));
        lastRotError = deltaAngle;
        double transBallOrth = egoBallPos->length() * mc.motion.rotation; //may be negative!
        double transBallTo = min(1000.0, ballVel2.length()); //Math.Max(ballPos.Distance(),ballVel2.Distance());
        auto driveTo = egoBallPos->rotateZ(-M_PI / 2.0);
        driveTo = driveTo.normalize() * transBallOrth;
        driveTo = driveTo + egoBallPos->normalize() * transBallTo;
        if (driveTo.length() > maxVel)
        {
            driveTo = driveTo.normalize() * maxVel;
        }
        mc.motion.angle = driveTo.angleZ();
        mc.motion.translation = driveTo.length();

        send(mc);

        /*PROTECTED REGION END*/
    }
    void AlignAndPassRapid::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1436269063295) ENABLED START*/ //Add additional options here
        teamMatePlanName.clear();
        teamMateTaskName.clear();

        int iter = 0;
        stringstream ss;
        stringstream ss2;
        string tmp;
        string tmp2;
        bool success = true;
        try
        {
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
            // has to be devided by 2 because our parameter is for setting the whole angle (left and right of the pass vector)
            success &= getParameter("FreeOppAngle", tmp);
            if (success)
            {
                this->freeOppAngle = stod(tmp) / 2;
                this->ratio = tan(freeOppAngle);
            }
            success &= getParameter("PassCorridorWidth", tmp);
            if (success)
            {
                this->passCorridorWidth = stod(tmp);
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
            success &= getParameter("MaxPassDist", tmp);
            if (success)
            {
                this->maxPassDist = stod(tmp);
            }
            success &= getParameter("DistToFieldBorder", tmp);
            if (success)
            {
                this->distToFieldBorder = stod(tmp);
            }
            this->minCloserOffset = (*this->sc)["Behaviour"]->get<double>("Pass", "MinCloserOffset", NULL);
            this->closerFactor = (*this->sc)["Behaviour"]->get<double>("Pass", "CloserFactor", NULL);
            this->closerFactor2 = (*this->sc)["Behaviour"]->get<double>("Pass", "CloserFactor2", NULL);
            this->arrivalTimeOffset = (*this->sc)["Behaviour"]->get<double>("Pass", "ArrivalTimeOffset", NULL);

            //Align Params
            this->maxVel = (*this->sc)["Behaviour"]->get<double>("Behaviour", "MaxSpeed", NULL);
            this->pRot = (*this->sc)["Dribble"]->get<double>("AlignAndPass", "RotationP", NULL);
            this->dRot = (*this->sc)["Dribble"]->get<double>("AlignAndPass", "RotationD", NULL);
            this->minRot = (*this->sc)["Dribble"]->get<double>("AlignAndPass", "MinRotation", NULL);
            this->maxRot = (*this->sc)["Dribble"]->get<double>("AlignAndPass", "MaxRotation", NULL);
            this->accel = (*this->sc)["Dribble"]->get<double>("AlignAndPass", "ReceiverRobotAcceleration", NULL);
            this->ballRadius = (*this->sc)["Rules"]->get<double>("Rules.BallRadius", NULL);
            lastRotError = 0;
        }
        catch (exception& e)
        {
            cerr << "Could not cast the parameter properly" << endl;
        }
        if (!success)
        {
            cerr << "AAPR: Parameter does not exist" << endl;
        }
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1436269063295) ENABLED START*/ //Add additional methods here
    void AlignAndPassRapid::findBestPassPoint(double cf, geometry::CNPointAllo passPoint,
                                              geometry::CNPointAllo receiver, shared_ptr<msl::VoronoiNet> vNet,
                                              geometry::CNPositionAllo teamMatePos, int teamMateId)
    {
        double currPassUtility = 0;
        auto rcv2PassPoint = passPoint - receiver;
        double rcv2PassPointDist = rcv2PassPoint.length();
        double factor = closerFactor;
        if (factor * rcv2PassPointDist < minCloserOffset)
        {
            factor = factor * rcv2PassPointDist;
        }
        else
        {
            factor = rcv2PassPointDist - minCloserOffset;
        }
        factor = max(factor, 0.0);
        if (rcv2PassPoint.x != 0 && rcv2PassPoint.y != 0)
        {
            passPoint = receiver + rcv2PassPoint.normalize() * factor;
        }
        else
        {
            passPoint = receiver;
        }
#ifdef DBM_DEBUG
        msl_helper_msgs::DebugPoint dbp;
        dbp.point.x = passPoint.x;
        dbp.point.y = passPoint.y;
        dbp.radius = 0.3;
        dbm->points.push_back(dbp);
#endif
        if (wm->field->isInsideField(passPoint, distToFieldBorder) // pass point must be inside the field with distance to side line of 1.5 metre
        && !wm->field->isInsidePenalty(passPoint, 0.0) && alloBall->distanceTo(passPoint) < maxPassDist // max dist to pass point
        && alloBall->distanceTo(passPoint) > minPassDist // min dist to pass point
                )
        {

            // min dist to opponent
            auto obs = vNet->getOpponentPositions();
            bool opponentTooClose = false;
            if (obs != nullptr)
            {
                for (int i = 0; i < obs->size(); i++)
                {
                    if (obs->at(i).distanceTo(passPoint) < minOppDist)
                    {
                        opponentTooClose = true;
                        break;
                    }
                }
            }
            if (opponentTooClose)
            {
#ifdef DBM_DEBUG
                dbm->points.at(dbm->points.size() - 1).red = 0.2 * 255.0;
                dbm->points.at(dbm->points.size() - 1).green = 0.2 * 255.0;
                dbm->points.at(dbm->points.size() - 1).blue = 0.2 * 255.0;
#endif
                return;
            }
            //small angle to turn to pass point
            if (geometry::absDeltaAngle(
                    alloPos->theta + M_PI,
                    (passPoint - geometry::CNVecAllo(alloPos->x, alloPos->y)).angleZ() > maxTurnAngle))
            {
#ifdef DBM_DEBUG
                dbm->points.at(dbm->points.size() - 1).red = 0.4 * 255.0;
                dbm->points.at(dbm->points.size() - 1).green = 0.4 * 255.0;
                dbm->points.at(dbm->points.size() - 1).blue = 0.4 * 255.0;
#endif
                return;
            }
            // some calculation to check whether any opponent is inside the pass vector triangle
            auto ball2PassPoint = passPoint - *alloBall;
            double passLength = ball2PassPoint.length();
            auto ball2PassPointOrth = geometry::CNVecAllo(-ball2PassPoint.y, ball2PassPoint.x).normalize() * ratio
                    * passLength;
            auto left = passPoint + ball2PassPointOrth;
            auto right = passPoint - ball2PassPointOrth;
            auto tmpBall = *alloBall;
            if (!outsideTriangle(tmpBall, right, left, ballRadius, vNet->getObstaclePositions())
                    && !outsideCorridore(tmpBall, passPoint, this->passCorridorWidth, vNet->getObstaclePositions()))
            {
#ifdef DBM_DEBUG
                dbm->points.at(dbm->points.size() - 1).red = 0.6 * 255.0;
                dbm->points.at(dbm->points.size() - 1).green = 0.6 * 255.0;
                dbm->points.at(dbm->points.size() - 1).blue = 0.6 * 255.0;
#endif
                return;
            }

            // no opponent was in dangerous distance to our pass vector, now check our teammates with other parameters
            if (!outsideCorridoreTeammates(tmpBall, passPoint, this->ballRadius * 4, *matePoses))
            {
#ifdef DBM_DEBUG
                dbm->points.at(dbm->points.size() - 1).red = 0.8 * 255.0;
                dbm->points.at(dbm->points.size() - 1).green = 0.8 * 255.0;
                dbm->points.at(dbm->points.size() - 1).blue = 0.8 * 255.0;
#endif
                return;
            }
            else
            {
                found = true;

#ifdef DBM_DEBUG
                dbm->points.at(dbm->points.size() - 1).red = 255;
                dbm->points.at(dbm->points.size() - 1).green = 255;
                dbm->points.at(dbm->points.size() - 1).blue = 255;
#endif

                //this.SuccessStatus = true;
                //Here we have to pick the best one...
                currPassUtility = 0;

                currPassUtility += 1.0 - 2.0 * abs(passPoint.y) / wm->field->getFieldWidth();

                currPassUtility += (wm->field->getFieldLength() / 2.0 + passPoint.x) / wm->field->getFieldLength();

                if (currPassUtility > bestPassUtility)
                {
#ifdef DBM_DEBUG
                    best_point = dbm->points.size() - 1;
#endif
                    alloAimPoint = passPoint;
                    bestPassUtility = currPassUtility;
                    bestAoc = teamMatePos.getPoint();
                    bestTeamMateId = teamMateId;
                }

            }
        }
    }

    bool AlignAndPassRapid::outsideCorridore(geometry::CNPointAllo ball, geometry::CNPointAllo passPoint,
                                             double passCorridorWidth, shared_ptr<vector<geometry::CNPointAllo>> points)
    {
        for (int i = 0; i < points->size(); i++)
        {
            auto point = points->at(i);
            if (geometry::distancePointToLineSegment2(point.x, point.y, ball, passPoint) < passCorridorWidth)
            {
                return false;
            }
        }
        return true;
    }

    bool AlignAndPassRapid::outsideCorridoreTeammates(geometry::CNPointAllo ball, geometry::CNPointAllo passPoint,
                                                      double passCorridorWidth, vector<geometry::CNPointAllo> points)
    {
        for (int i = 0; i < points.size(); i++)
        {
            auto point = points.at(i);
            if (geometry::distancePointToLineSegment2(point.x, point.y, ball, passPoint) < passCorridorWidth
                    && ball.distanceTo(points.at(i)) < ball.distanceTo(passPoint) - 100)
            {
                return false;
            }
        }
        return true;
    }

    bool AlignAndPassRapid::outsideTriangle(geometry::CNPointAllo a, geometry::CNPointAllo b, geometry::CNPointAllo c,
                                            double tolerance, shared_ptr<vector<geometry::CNPointAllo>> points)
    {
        auto a2b = b - a;
        auto b2c = c - b;
        auto c2a = a - c;
        for (int i = 0; i < points->size(); i++)
        {
            auto p = points->at(i);
            auto a2p = p - a;
            auto b2p = p - b;
            auto c2p = p - c;

            if ((a2p.x * a2b.y - a2p.y * a2b.x) / a2p.normalize().length() < tolerance
                    && (b2p.x * b2c.y - b2p.y * b2c.x) / b2p.normalize().length() < tolerance
                    && (c2p.x * c2a.y - c2p.y * c2a.x) / c2p.normalize().length() < tolerance)
            {
                return false;
            }

        }
        return true;
    }

    double AlignAndPassRapid::minFree(double angle, double width, shared_ptr<const vector<double> > dstscan)
    {
        double sectorWidth = 2.0 * M_PI / dstscan->size();
        int startSector = mod((int)floor(angle / sectorWidth), dstscan->size());
        double minfree = dstscan->at(startSector);
        double dist, dangle;
        for (int i = 1; i < dstscan->size() / 4; i++)
        {
            dist = dstscan->at(mod((startSector + i), dstscan->size()));
            dangle = sectorWidth * i;
            if (abs(dist * sin(dangle)) < width)
            {
                minfree = min(minfree, abs(dist * cos(dangle)));
            }

            dist = dstscan->at(mod((startSector - i), dstscan->size()));
            if (abs(dist * sin(dangle)) < width)
            {
                minfree = min(minfree, abs(dist * cos(dangle)));
            }

        }
        return minfree;
    }

    int AlignAndPassRapid::mod(int x, int y)
    {
        int z = x % y;
        if (z < 0)
            return y + z;
        else
            return z;
    }
/*PROTECTED REGION END*/
} /* namespace alica */
