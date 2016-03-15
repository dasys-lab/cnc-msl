using namespace std;
#include "Plans/Attack/AlignAndPassRapid.h"

/*PROTECTED REGION ID(inccpp1436269063295) ENABLED START*/ //Add additional includes here
#include "msl_helper_msgs/PassMsg.h"
#include "pathplanner/VoronoiNet.h"
#include "pathplanner/PathProxy.h"
#define DBM_DEBUG 1
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
        this->arrivalTimeOffset = NAN;
        this->maxVel = 2000;
        this->pRot = 2.1;
        this->dRot = 0.0;
        this->lastRotError = 0;
        this->minRot = 0.1;
        this->maxRot = M_PI * 4;
        this->accel = 2000;
        this->sc = supplementary::SystemConfig::getInstance();
        this->alloAimPoint = nullptr;
        this->field = msl::MSLFootballField::getInstance();
        this->pathProxy = msl::PathProxy::getInstance();
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
        shared_ptr < geometry::CNPosition > alloPos = this->wm->rawSensorData.getOwnPositionVision();
        if (alloPos == nullptr)
        {
//			mc = DriveHelper.DriveRandomly(500,WM);
//			Send(mc);
            cout << "AAPR: OwnPos is null" << endl;
            ;
            return;
        }

        shared_ptr < geometry::CNPoint2D > egoBallPos = this->wm->ball.getEgoBallPosition();
        if (egoBallPos == nullptr)
        {
            cout << "AAPR: Ego Ball is null" << endl;
            return;
        }

        shared_ptr < geometry::CNPoint2D > alloBall = this->wm->ball.getAlloBallPosition();
        if (alloBall == nullptr)
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

        auto vNet = this->wm->pathPlanner.getCurrentVoronoiNet();
        if (vNet == nullptr)
        {
            cout << "AAPR: VNet is null!" << endl;
            return;
        }

		auto matePoses = wm->robots.teammates.getTeammatesAlloClustered();
		if(matePoses == nullptr)
		{
			cout << "matePoses == nullptr" << endl;
            return;
		}
        for(auto i=matePoses->begin(); i!=matePoses->end(); i++) {
        	if((*i)->distanceTo(alloPos) < 100) {
        		matePoses->erase(i);
        		break;
        	}
        }

        try
        {
            double bestPassUtility = numeric_limits<double>::min();
            double currPassUtility = 0;
            int bestTeamMateId = -1;
            shared_ptr < geometry::CNPoint2D > bestPassVNode = nullptr;
            shared_ptr < geometry::CNPoint2D > bestAoc = nullptr;
            bool found = false;

            #ifdef DBM_DEBUG
            int best_point = -1;
            msl_helper_msgs::DebugMsg dbm;
            #endif
            for (int teamMateId : this->teamMateIds)
            {
                shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> vertices = vNet->getTeamMateVerticesCNPoint2D(
                        teamMateId);
                shared_ptr < geometry::CNPosition > teamMatePos = wm->robots.teammates.getTeamMatePosition(teamMateId);
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

                    #ifdef DBM_DEBUG
                    msl_helper_msgs::DebugPoint dbp;
                    dbp.point.x = passPoint->x;
                    dbp.point.y = passPoint->y;
                    dbm.points.push_back(dbp);
                    #endif
                    if (field->isInsideField(passPoint, distToFieldBorder) // pass point must be inside the field with distance to side line of 1.5 metre
                    && !field->isInsidePenalty(passPoint, 0.0) && alloBall->distanceTo(passPoint) < maxPassDist // max dist to pass point
                    && alloBall->distanceTo(passPoint) > minPassDist // min dist to pass point
                            )
                    {

                        // min dist to opponent
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
							#ifdef DBM_DEBUG
                        	dbm.points.at(dbm.points.size()-1).red = 0.2*255.0;
                        	dbm.points.at(dbm.points.size()-1).green = 0.2*255.0;
                        	dbm.points.at(dbm.points.size()-1).blue = 0.2*255.0;
							#endif
                            continue;
                        }
//						if ((vNodes[i].tri.p[0].ident == -1 && vNodes[i].tri.p[0].DistanceTo(passPoint) < minOppDist)
//								|| (vNodes[i].tri.p[1].ident == -1
//										&& vNodes[i].tri.p[1].DistanceTo(passPoint) < minOppDist)
//								|| (vNodes[i].tri.p[2].ident == -1
//										&& vNodes[i].tri.p[2].DistanceTo(passPoint) < minOppDist))
//						{
//							continue;
//						}

                        //small angle to turn to pass point
                        if (geometry::GeometryCalculator::absDeltaAngle(
                                alloPos->theta + M_PI,
                                (passPoint - make_shared < geometry::CNPoint2D > (alloPos->x, alloPos->y))->angleTo())
                                > maxTurnAngle)
                        {
                        	#ifdef DBM_DEBUG
                        	dbm.points.at(dbm.points.size()-1).red = 0.4*255.0;
                        	dbm.points.at(dbm.points.size()-1).green = 0.4*255.0;
                        	dbm.points.at(dbm.points.size()-1).blue = 0.4*255.0;
							#endif
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
                        	#ifdef DBM_DEBUG
                        	dbm.points.at(dbm.points.size()-1).red = 0.6*255.0;
                        	dbm.points.at(dbm.points.size()-1).green = 0.6*255.0;
                        	dbm.points.at(dbm.points.size()-1).blue = 0.6*255.0;
							#endif
                            continue;
                        }

                        // no opponent was in dangerous distance to our pass vector, now check our teammates with other parameters
                        if (!outsideCorridoreTeammates(alloBall, passPoint, this->ballRadius * 4,
                                                       matePoses))
                        {
							#ifdef DBM_DEBUG
                        	dbm.points.at(dbm.points.size()-1).red = 0.8*255.0;
                        	dbm.points.at(dbm.points.size()-1).green = 0.8*255.0;
                        	dbm.points.at(dbm.points.size()-1).blue = 0.8*255.0;
							#endif
                            continue;
                        }
                        else
                        {
                            found = true;

							#ifdef DBM_DEBUG
                        	dbm.points.at(dbm.points.size()-1).red = 255;
                        	dbm.points.at(dbm.points.size()-1).green = 255;
                        	dbm.points.at(dbm.points.size()-1).blue = 255;
							#endif

                            //this.SuccessStatus = true;
                            //Here we have to pick the best one...
                            currPassUtility = 0;

                            currPassUtility += 1.0 - 2.0 * abs(passPoint->y) / field->FieldWidth;

                            currPassUtility += (field->FieldLength / 2.0 + passPoint->x) / field->FieldLength;

                            if (currPassUtility > bestPassUtility)
                            {
                            	#ifdef DBM_DEBUG
                            	best_point = dbm.points.size()-1;
								#endif
                                alloAimPoint = passPoint;
                                bestPassUtility = currPassUtility;
                                bestAoc = make_shared < geometry::CNPoint2D > (teamMatePos->x, teamMatePos->y);
                                bestTeamMateId = teamMateId;
                            }

                        }
                    }
                }
            }
			#ifdef DBM_DEBUG
			dbm.points.at(best_point).red = 0;
			dbm.points.at(best_point).green = 0;
            send(dbm);
			#endif

            if (!found)
            { // No Pass point found, so return everything
                this->success = true;
                cout << "AAPR: No valid pass point found! SuccessStatus: " << this->success << endl;
                return;
            }
            //Turn to goal...
            shared_ptr < geometry::CNVelocity2D > ballVel = this->wm->ball.getVisionBallVelocity();
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
            shared_ptr < geometry::CNPoint2D > aimPoint = alloAimPoint->alloToEgo(*alloPos);
            double aimAngle = aimPoint->angleTo();
            double ballAngle = egoBallPos->angleTo();
            double deltaAngle = geometry::GeometryCalculator::deltaAngle(ballAngle, aimAngle);
            if (abs(deltaAngle) < M_PI / 36)
            { // +/-5 degree
              //Kick && PassMsg
                msl_helper_msgs::PassMsg pm;
                msl_msgs::Point2dInfo pinf;
                // Distance to aim point * direction of our kicker = actual pass point destination
                double dist = aimPoint->length();
                shared_ptr < geometry::CNPoint2D > dest = make_shared < geometry::CNPoint2D > (-dist, 0);
                dest = dest->egoToAllo(*alloPos);
                pinf.x = dest->x;
                pinf.y = dest->y;
                pm.destination = pinf;
                pinf = msl_msgs::Point2dInfo();
                pinf.x = alloPos->x;
                pinf.y = alloPos->y;
                pm.origin = pinf;
                pm.receiverID = bestTeamMateId;
                msl_actuator_msgs::KickControl km;
                km.enabled = true;
                km.kicker = 1; //(ushort)KickHelper.KickerToUseIndex(egoBallPos->angleTo());

                shared_ptr < geometry::CNPoint2D > goalReceiverVec = dest - make_shared < geometry::CNPoint2D
                        > (bestAoc->x, bestAoc->y);
                double v0 = 0;
                double distReceiver = goalReceiverVec->length();
                double estimatedTimeForReceiverToArrive = (sqrt(2 * accel * distReceiver + v0 * v0) - v0) / accel;
                pm.validFor = (uint)(estimatedTimeForReceiverToArrive * 1000.0 + 300.0); // this is sparta!
                if (closerFactor < 0.01)
                {
                    km.power = (ushort)wm->kicker.getKickPowerPass(aimPoint->length());
                }
                else
                {
                    km.power = (ushort)wm->kicker.getPassKickpower(
                            dist, estimatedTimeForReceiverToArrive + arrivalTimeOffset);
                }
                if(wm->isUsingSimulator())
                {
                	km.power = km.power * 1.5;
                }

                send(km);
                if (wm->kicker.lowShovelSelected)
                {
                    send(pm);
                }

            }
            auto dstscan = this->wm->rawSensorData.getDistanceScan();
            if (dstscan != nullptr && dstscan->size() != 0)
            {
                double distBeforeBall = minFree(egoBallPos->angleTo(), 200, dstscan);
                if (distBeforeBall < 250)
                    this->failure = true;
            }
            mc = msl_actuator_msgs::MotionControl();
            mc.motion.rotation = deltaAngle * pRot + (deltaAngle - lastRotError) * dRot;
            double sign = geometry::GeometryCalculator::sgn(mc.motion.rotation);
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

        }
        catch (exception& e)
        {
            throw e;
        }

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
    bool AlignAndPassRapid::outsideCorridore(shared_ptr<geometry::CNPoint2D> ball,
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

    bool AlignAndPassRapid::outsideCorridoreTeammates(shared_ptr<geometry::CNPoint2D> ball,
                                                      shared_ptr<geometry::CNPoint2D> passPoint,
                                                      double passCorridorWidth,
                                                      shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> points)
    {
        for (int i = 0; i < points->size(); i++)
        {
            if (geometry::GeometryCalculator::distancePointToLineSegment(points->at(i)->x, points->at(i)->y, ball, passPoint)
            < passCorridorWidth && ball->distanceTo(points->at(i)) < ball->distanceTo(passPoint) - 100)
            {
                return false;
            }
        }
        return true;
    }

    bool AlignAndPassRapid::outsideTriangle(shared_ptr<geometry::CNPoint2D> a, shared_ptr<geometry::CNPoint2D> b,
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

            if ((a2p->x * a2b->y - a2p->y * a2b->x) / a2p->normalize()->length() < tolerance
            && (b2p->x * b2c->y - b2p->y * b2c->x) / b2p->normalize()->length() < tolerance
            && (c2p->x * c2a->y - c2p->y * c2a->x) / c2p->normalize()->length() < tolerance)
            {
                return false;
            }

        }
        return true;
    }

    double AlignAndPassRapid::minFree(double angle, double width, shared_ptr<vector<double> > dstscan)
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
