using namespace std;
#include "Plans/Standards/Own/ThrowIn/ThrowInPass.h"

/*PROTECTED REGION ID(inccpp1462363192018) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include "engine/model/EntryPoint.h"
#include "engine/RunningPlan.h"
#include "engine/Assignment.h"
#include "engine/model/Plan.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1462363192018) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    ThrowInPass::ThrowInPass() :
            DomainBehaviour("ThrowInPass")
    {
        /*PROTECTED REGION ID(con1462363192018) ENABLED START*/ //Add additional options here
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
        this->maxVel = 2000;
        this->pRot = 2.1;
        this->dRot = 0.0;
        this->lastRotError = 0;
        this->minRot = 0.1;
        this->maxRot = M_PI * 4;
        this->accel = 2000;
        this->arrivalTimeOffset = NAN;
        /*PROTECTED REGION END*/
    }
    ThrowInPass::~ThrowInPass()
    {
        /*PROTECTED REGION ID(dcon1462363192018) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void ThrowInPass::run(void* msg)
    {
        /*PROTECTED REGION ID(run1462363192018) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision(); // actually ownPosition corrected
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball->getEgoBallPosition();
        // return if necessary information is missing
        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }
        canPass = false;
        shared_ptr < geometry::CNPoint2D > alloTarget = nullptr;
        shared_ptr < geometry::CNPoint2D > alloBall = egoBallPos->egoToAllo(*ownPos);

        int recId = -1;
        int aRecId = -1;
        EntryPoint* ep = getParentEntryPoint(teamMateTaskName1);
        if (ep != nullptr)
        {
            // get the plan in which the behavior is running
            auto parent = this->runningPlan->getParent().lock();
            if (parent == nullptr)
            {
                cout << "parent null" << endl;
                return;
            }
            // get robot ids of robots in found entry point
            shared_ptr<vector<int>> ids = parent->getAssignment()->getRobotsWorking(ep);
            // exactly one robot is receiver
            if (ids->size() > 0 && ids->at(0) != -1)
            {
                // get receiver position by id
                auto pos = wm->robots->teammates.getTeamMatePosition(ids->at(0));
                if (pos != nullptr)
                {
                    recId = ids->at(0);
                    recPos1 = make_shared < geometry::CNPoint2D > (pos->x, pos->y);
                }
                else
                {
                    recPos1 = nullptr;
                }
            }
        }

        EntryPoint* ep2 = getParentEntryPoint(teamMateTaskName2);
        if (ep2 != nullptr)
        {
            // get the plan in which the behavior is running
            auto parent = this->runningPlan->getParent().lock();
            if (parent == nullptr)
            {
                cout << "parent null" << endl;
                return;
            }
            // get robot ids of robots in found entry point
            shared_ptr<vector<int>> ids = parent->getAssignment()->getRobotsWorking(ep2);
            // exactly one robot is receiver
            if (ids->size() > 0 && ids->at(0) != -1)
            {
                // get receiver position by id
                auto pos = wm->robots->teammates.getTeamMatePosition(ids->at(0));
                if (pos != nullptr)
                {
                    aRecId = ids->at(0);
                    recPos2 = make_shared < geometry::CNPoint2D > (pos->x, pos->y);
                }
                else
                {
                    recPos2 = nullptr;
                }
            }
        }

        if (recPos1 == nullptr && recPos2 == nullptr)
        {
            return;
        }
        shared_ptr < geometry::CNPoint2D > bestAoc = nullptr;
        if (recId != -1)
        {
            double bestPassUtility = numeric_limits<double>::min();
            double currPassUtility = 0;
            shared_ptr < geometry::CNPoint2D > bestPassVNode = nullptr;
            auto vNet = this->wm->pathPlanner->getCurrentVoronoiNet();
            shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> vertices = vNet->getTeamMateVerticesCNPoint2D(recId);
            shared_ptr < geometry::CNPosition > teamMatePos = wm->robots->teammates.getTeamMatePosition(recId);

            if (vertices == nullptr || teamMatePos == nullptr)
            {
                cout << "SAAG2R: vertices or teamMatePos == nullptr!" << endl;
                return;
            }
            for (int i = 0; i < vertices->size(); i++)
            {
                // make the passpoints closer to the receiver
                shared_ptr < geometry::CNPoint2D > passPoint = vertices->at(i);

                shared_ptr < geometry::CNPoint2D > receiver = make_shared < geometry::CNPoint2D
                        > (teamMatePos->x, teamMatePos->y);
                shared_ptr < geometry::CNPoint2D > rcv2PassPoint = passPoint - receiver;
                double rcv2PassPointDist = rcv2PassPoint->length();
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
                passPoint = receiver + rcv2PassPoint->normalize() * factor;
                if (wm->field->isInsideField(passPoint, distToFieldBorder) // pass point must be inside the field with distance to side line of 1.5 metre
                && !wm->field->isInsidePenalty(passPoint, 0.0) && alloBall->distanceTo(passPoint) < maxPassDist // max dist to pass point
                && alloBall->distanceTo(passPoint) > minPassDist // min dist to pass point
                        )
                {

                    // min dist to opponent
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
                    if (geometry::absDeltaAngle(
                            ownPos->theta + M_PI,
                            (passPoint - make_shared < geometry::CNPoint2D > (ownPos->x, ownPos->y))->angleTo())
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
                    auto matePoses = wm->robots->teammates.getTeammatesAlloClustered();
                    if (matePoses != nullptr
                            && !outsideCorridoreTeammates(alloBall, passPoint, this->ballRadius * 4, matePoses))
                    {
                        continue;
                    }
                    else
                    {
                        canPass = true;
                        currPassUtility = 0;

                        currPassUtility += 1.0 - 2.0 * abs(passPoint->y) / wm->field->getFieldWidth();

                        currPassUtility += (wm->field->getFieldLength() / 2.0 + passPoint->x)
                                / wm->field->getFieldLength();

                        if (currPassUtility > bestPassUtility)
                        {
                            alloAimPoint = passPoint;
                            bestPassUtility = currPassUtility;
                            bestAoc = make_shared < geometry::CNPoint2D > (teamMatePos->x, teamMatePos->y);
                        }
                    }
                }
            }
        }
        int bestReceiverId = -1;
        if (canPass)
        {
            alloTarget = recPos1;
            bestReceiverId = recId;
        }
        else
        {
            alloTarget = recPos2;
            bestReceiverId = aRecId;
            bestAoc = make_shared < geometry::CNPoint2D > (alloTarget->x, alloTarget->y);
        }
        MotionControl mc;
        shared_ptr < geometry::CNVelocity2D > ballVel = this->wm->ball->getVisionBallVelocity();
        shared_ptr < geometry::CNPoint2D > ballVel2;
        if (ballVel == nullptr)
        {
            ballVel2 = make_shared < geometry::CNPoint2D > (0, 0);
        }
        else if (ballVel->length() > 5000)
        {
            shared_ptr < geometry::CNVelocity2D > v = ballVel->normalize() * 5000;
            ballVel2 = make_shared < geometry::CNPoint2D > (v->x, v->y);
        }
        else
        {
            ballVel2 = make_shared < geometry::CNPoint2D > (ballVel->x, ballVel->y);
        }
        shared_ptr < geometry::CNPoint2D > aimPoint = alloAimPoint->alloToEgo(*ownPos);
        double aimAngle = aimPoint->angleTo();
        double ballAngle = egoBallPos->angleTo();
        double deltaAngle = geometry::deltaAngle(ballAngle, aimAngle);
        if (abs(deltaAngle) < M_PI / 36)
        { // +/-5 degree
          //Kick && PassMsg
            msl_helper_msgs::PassMsg pm;
            msl_msgs::Point2dInfo pinf;
            // Distance to aim point * direction of our kicker = actual pass point destination
            double dist = aimPoint->length();
            shared_ptr < geometry::CNPoint2D > dest = make_shared < geometry::CNPoint2D > (-dist, 0);
            dest = dest->egoToAllo(*ownPos);
            pinf.x = dest->x;
            pinf.y = dest->y;
            pm.destination = pinf;
            pinf = msl_msgs::Point2dInfo();
            pinf.x = ownPos->x;
            pinf.y = ownPos->y;
            pm.origin = pinf;
            pm.receiverID = bestReceiverId;
            msl_actuator_msgs::KickControl km;
            km.enabled = true;
            km.kicker = 1; //(ushort)KickHelper.KickerToUseIndex(egoBallPos->angleTo());

            shared_ptr < geometry::CNPoint2D > goalReceiverVec = dest - make_shared < geometry::CNPoint2D
                    > (bestAoc->x, bestAoc->y);
            double v0 = 0;
            double distReceiver = goalReceiverVec->length();
            double estimatedTimeForReceiverToArrive = (sqrt(2 * accel * distReceiver + v0 * v0) - v0) / accel;
            //considering network delay and reaction time 1s?:
            estimatedTimeForReceiverToArrive += 1.0;
            pm.validFor = (unsigned long long)(estimatedTimeForReceiverToArrive * 1000000000.0 + 300000000.0); // this is sparta!
            if (closerFactor < 0.01)
            {
                km.power = (ushort)wm->kicker->getKickPowerPass(aimPoint->length());
            }
            else
            {
                km.power = (ushort)wm->kicker->getPassKickpower(dist,
                                                                estimatedTimeForReceiverToArrive + arrivalTimeOffset);
            }

            send(km);
            if (wm->kicker->lowShovelSelected)
            {
                send(pm);
            }

        }
        auto dstscan = this->wm->rawSensorData->getDistanceScan();
        if (dstscan != nullptr && dstscan->size() != 0)
        {
            double distBeforeBall = minFree(egoBallPos->angleTo(), 200, dstscan);
            if (distBeforeBall < 250)
                this->setFailure(true);
        }
        mc = msl_actuator_msgs::MotionControl();
        mc.motion.rotation = deltaAngle * pRot + (deltaAngle - lastRotError) * dRot;
        double sign = geometry::sgn(mc.motion.rotation);
        mc.motion.rotation = sign * min(this->maxRot, max(abs(mc.motion.rotation), this->minRot));
        lastRotError = deltaAngle;
        double transBallOrth = egoBallPos->length() * mc.motion.rotation; //may be negative!
        double transBallTo = min(1000.0, ballVel2->length()); //Math.Max(ballPos.Distance(),ballVel2.Distance());
        shared_ptr < geometry::CNPoint2D > driveTo = egoBallPos->rotate(-M_PI / 2.0);
        driveTo = driveTo->normalize() * transBallOrth;
        driveTo = driveTo + egoBallPos->normalize() * transBallTo;
        if (driveTo->length() > maxVel)
        {
            driveTo = driveTo->normalize() * maxVel;
        }
        mc.motion.angle = driveTo->angleTo();
        mc.motion.translation = driveTo->length();

        send(mc);

        /*PROTECTED REGION END*/
    }
    void ThrowInPass::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1462363192018) ENABLED START*/ //Add additional options here
        auto sc = supplementary::SystemConfig::getInstance();
        this->minCloserOffset = (*this->sc)["Behaviour"]->get<double>("Pass", "MinCloserOffset", NULL);
        this->closerFactor = (*this->sc)["Behaviour"]->get<double>("Pass", "CloserFactor", NULL);
        this->ballRadius = (*this->sc)["Rules"]->get<double>("Rules.BallRadius", NULL);
        this->freeOppAngle = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "freeOppAngle", NULL) / 2;
        this->ratio = tan(this->freeOppAngle);
        this->passCorridorWidth = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "passCorridorWidth", NULL);
        this->maxTurnAngle = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "maxTurnAngle", NULL);
        this->minOppDist = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "minOppDist", NULL);
        this->minPassDist = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "minPassDist", NULL);
        this->maxPassDist = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "maxPassDist", NULL);
        this->distToFieldBorder = (*this->sc)["Behaviour"]->get<double>("ThrowIn", "distToFieldBorder", NULL);
        this->maxVel = (*this->sc)["Behaviour"]->get<double>("Behaviour", "MaxSpeed", NULL);
        this->pRot = (*this->sc)["Dribble"]->get<double>("AlignAndPass", "RotationP", NULL);
        this->dRot = (*this->sc)["Dribble"]->get<double>("AlignAndPass", "RotationD", NULL);
        this->minRot = (*this->sc)["Dribble"]->get<double>("AlignAndPass", "MinRotation", NULL);
        this->maxRot = (*this->sc)["Dribble"]->get<double>("AlignAndPass", "MaxRotation", NULL);
        this->accel = (*this->sc)["Dribble"]->get<double>("AlignAndPass", "ReceiverRobotAcceleration", NULL);
        this->arrivalTimeOffset = (*this->sc)["Behaviour"]->get<double>("Pass", "ArrivalTimeOffset", NULL);
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
            cerr << "SA2P: Parameter does not exist" << endl;
        }

        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1462363192018) ENABLED START*/ //Add additional methods here
    bool ThrowInPass::outsideCorridore(shared_ptr<geometry::CNPoint2D> ball, shared_ptr<geometry::CNPoint2D> passPoint,
                                       double passCorridorWidth, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points)
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

    bool ThrowInPass::outsideCorridoreTeammates(shared_ptr<geometry::CNPoint2D> ball,
                                                shared_ptr<geometry::CNPoint2D> passPoint, double passCorridorWidth,
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

    bool ThrowInPass::outsideTriangle(shared_ptr<geometry::CNPoint2D> a, shared_ptr<geometry::CNPoint2D> b,
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

            if ((a2p->x * a2b->y - a2p->y * a2b->x) / a2p->normalize()->length()<tolerance
            && (b2p->x * b2c->y - b2p->y * b2c->x) / b2p->normalize()->length() < tolerance
            && (c2p->x * c2a->y - c2p->y * c2a->x) / c2p->normalize()->length() < tolerance)
            {
                return false;
            }

        }
        return true;
    }

    double ThrowInPass::minFree(double angle, double width, shared_ptr<vector<double> > dstscan)
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

    int ThrowInPass::mod(int x, int y)
    {
        int z = x % y;
        if (z < 0)
            return y + z;
        else
            return z;
    }

/*PROTECTED REGION END*/
} /* namespace alica */
