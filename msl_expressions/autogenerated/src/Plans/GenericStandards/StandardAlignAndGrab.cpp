//using namespace std;
//#include "Plans/GenericStandards/StandardAlignAndGrab.h"
//
///*PROTECTED REGION ID(inccpp1455888574532) ENABLED START*/ //Add additional includes here
//#include "msl_robot/robotmovement/RobotMovement.h"
//#include <RawSensorData.h>
//#include <Ball.h>
//#include <Robots.h>
//#include <GeometryCalculator.h>
//#include <msl_robot/MSLRobot.h>
//#include <msl_robot/kicker/Kicker.h>
//#include <MSLWorldModel.h>
///*PROTECTED REGION END*/
//namespace alica
//{
//    /*PROTECTED REGION ID(staticVars1455888574532) ENABLED START*/ //initialise static variables here
//    /*PROTECTED REGION END*/
//    StandardAlignAndGrab::StandardAlignAndGrab() :
//            DomainBehaviour("StandardAlignAndGrab")
//    {
//        /*PROTECTED REGION ID(con1455888574532) ENABLED START*/ //Add additional options here
//        this->maxTranslation = (*sc)["Behaviour"]->get<double>("StandardAlign.StandardSituationSpeed", NULL);
//        this->tol = (*sc)["Behaviour"]->get<double>("StandardAlign.AlignTolerance", NULL);
//        this->trans = (*sc)["Behaviour"]->get<double>("StandardAlign.AlignSpeed", NULL);
//        this->minTol = (*sc)["Behaviour"]->get<double>("StandardAlign.MinAlignTolerance", NULL);
//
//        query = make_shared<msl::MovementQuery>();
//        /*PROTECTED REGION END*/
//    }
//    StandardAlignAndGrab::~StandardAlignAndGrab()
//    {
//        /*PROTECTED REGION ID(dcon1455888574532) ENABLED START*/ //Add additional options here
//        /*PROTECTED REGION END*/
//    }
//    void StandardAlignAndGrab::run(void* msg)
//    {
//        /*PROTECTED REGION ID(run1455888574532) ENABLED START*/ //Add additional options here
//        msl::RobotMovement rm;
//        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision(); // actually ownPosition corrected
//        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball->getEgoBallPosition();
//        // return if necessary information is missing
//        if (ownPos == nullptr || egoBallPos == nullptr)
//        {
//            return;
//        }
//        shared_ptr < geometry::CNPoint2D > alloBall = egoBallPos->egoToAllo(*ownPos);
//
//        if (receiver == nullptr)
//        {
//            receiver = getHigherEntryPoint(planName, teamMateTaskName);
//        }
//
//        msl_actuator_msgs::MotionControl mc;
//        if (egoBallPos->length() > 900)
//        {
//            // Drive close to the ball, until dist < 900
//            // replaced with new moveToPointMethod
////            mc = msl::RobotMovement::moveToPointCarefully(egoBallPos, egoBallPos, 0, nullptr);
//            query->egoDestinationPoint = egoBallPos;
//            query->egoAlignPoint = egoBallPos;
//            mc = rm.moveToPoint(query);
//
//            cout << "SAAG: egoBallPos->length() > 900 ROT: \t" << mc.motion.rotation << endl;
//            send(mc);
//            return;
//        }
//
//        haveBall = wm->ball->haveBall();
//        if (!haveBall)
//        {
//            haveBallCounter = 0;
//        }
//
//        if (egoBallPos->length() > 450)
//        {
//            // Drive closer to the ball, but don't rotate
//            // replaced with new moveToPoint method
////            mc = msl::RobotMovement::moveToPointCarefully(egoBallPos, egoBallPos, 0, nullptr);
//            query->egoDestinationPoint = egoBallPos;
//            query->egoAlignPoint = egoBallPos;
//            mc = rm.moveToPoint(query);
//
//            mc.motion.rotation = 0;
//            mc.motion.translation = min(600.0, egoBallPos->length() / 1.66);
//            cout << "SAAG: egoBallPos->length() > 450 ROT: \t" << mc.motion.rotation << endl;
//            send(mc);
//            return;
//        }
//
//        double radian = egoBallPos->length();
//        double rot = this->trans / radian;
//
//        shared_ptr < geometry::CNPoint2D > egoMatePos;
//        shared_ptr < geometry::CNPosition > matePos;
//        if (receiver != nullptr)
//        {
//            auto robots = robotsInEntryPointOfHigherPlan(receiver);
//
//            if (robots != nullptr && robots->size() > 0)
//            {
//                matePos = wm->robots->teammates.getTeamMatePosition(robots->at(0));
//            }
//            if (matePos != nullptr)
//            {
//                egoMatePos = matePos->getPoint()->alloToEgo(*ownPos);
//                oldMatePos = egoMatePos->clone();
//            }
//            else if (oldMatePos != nullptr)
//            {
//                egoMatePos = oldMatePos->clone();
//            }
//        }
//
//        if (egoMatePos == nullptr)
//        {
//            egoMatePos = make_shared < geometry::CNPoint2D > (0, 0)->alloToEgo(*ownPos);
//        }
//
//        shared_ptr < geometry::CNPoint2D > direction = nullptr;
//
//        double dangle = geometry::deltaAngle(this->robot->kicker->kickerAngle, egoMatePos->angleTo());
//
//        double cross = egoMatePos->x * egoBallPos->y - egoMatePos->y * egoBallPos->x;
//        double fac = -(cross > 0 ? 1 : -1);
//        if (fabs(dangle) < 12.0 * M_PI / 180.0)
//        {
//            direction = egoBallPos->rotate(-fac * M_PI / 2.0)->normalize() * this->trans * 0.66;
//        }
//        else
//        {
//            direction = egoBallPos->rotate(-fac * M_PI / 2.0)->normalize() * this->trans;
//        }
//
//        double balldangle = geometry::deltaAngle(this->robot->kicker->kickerAngle, egoBallPos->angleTo());
//        if (egoBallPos->length() > 350 && fabs(dangle) > 35.0 * M_PI / 180.0)
//        {
//            mc.motion.angle = direction->angleTo();
//            mc.motion.translation = direction->length() * 1.6;
//            mc.motion.rotation = fac * rot * 1.6;
//            cout << "SAAG: egoBallPos->length() > 350 && fabs(dangle) > 35.0 * M_PI / 180.0 ROT: \t"
//                    << mc.motion.rotation << endl;
//            send(mc);
//            return;
//        }
//
//        if (!haveBall)
//        {
//            if (fabs(balldangle) > 20.0 * M_PI / 180.0)
//            {
//                mc.motion.rotation = (balldangle > 0 ? 1 : -1) * 0.8;
//                mc.motion.angle = M_PI;
//                mc.motion.translation = 100;
//                cout << "SAAG: fabs(balldangle) > 20.0 * M_PI / 180.0 ROT: \t" << mc.motion.rotation << endl;
//                send(mc);
//                return;
//            }
//            else
//            {
//                mc.motion.rotation = balldangle * 0.5;
//                mc.motion.angle = egoBallPos->angleTo();
//                mc.motion.translation = egoBallPos->length() * 1.5;
//                cout << "SAAG: fabs(balldangle) > 20.0 * M_PI / 180.0 else ROT: \t" << mc.motion.rotation << endl;
//                send(mc);
//                return;
//            }
//        }
//
//        angleIntErr += dangle;
//        mc.motion.angle = direction->angleTo();
//        mc.motion.translation = direction->length();
//        mc.motion.rotation = fac * rot * (2 * fabs(0.8 * dangle + 0.1 * angleIntErr + 2 * (dangle - oldAngleErr)));
//        oldAngleErr = dangle;
//        if (haveBall)
//        {
//            haveBallCounter++;
//            double runningTimeMS = (double)((wm->getTime() - startTime) / 1000000ul);
//            if (runningTimeMS > 9000)
//            {
//                mc.motion.angle = M_PI;
//                mc.motion.rotation = 0.0;
//                mc.motion.translation = 100.0;
////                cout << "SAAG: haveBall" << endl;
//                this->setSuccess(true);
//            }
//            else if (haveBallCounter > 6
//                    && ((runningTimeMS <= 4000.0 && fabs(dangle) < this->minTol)
//                            || fabs(dangle)
//                                    < this->minTol
//                                            + max(0.0, (this->tol - this->minTol) / (5000.0 / (runningTimeMS - 4000.0)))))
//            {
//                mc.motion.angle = M_PI;
//                mc.motion.rotation = 0.0;
//                mc.motion.translation = 100.0;
////                cout << "SAAG: haveBall esle if" << endl;
//                this->setSuccess(true);
//            }
//        }
//        cout << "SAAG: last mc ROT: \t" << mc.motion.rotation << endl;
//        send(mc);
//        /*PROTECTED REGION END*/
//    }
//    void StandardAlignAndGrab::initialiseParameters()
//    {
//        /*PROTECTED REGION ID(initialiseParameters1455888574532) ENABLED START*/ //Add additional options here
//        angleIntErr = 0;
//        oldAngleErr = 0;
//        haveBall = false;
//        haveBallCounter = 0;
//        delayKickCounter = 0;
//        oldMatePos.reset();
//        startTime = wm->getTime();
//
//        bool success = true;
//        try
//        {
//            success &= getParameter("planName", planName);
//            success &= getParameter("teamMateTaskName", teamMateTaskName);
//        }
//        catch (exception& e)
//        {
//            cerr << "Could not cast the parameter properly" << endl;
//        }
//        if (!success)
//        {
//            cerr << "StandardAlignAndGrab: Parameter does not exist" << endl;
//        }
//
//        receiver = getHigherEntryPoint(planName, teamMateTaskName);
//        if (receiver == nullptr)
//        {
//            cerr << "StdAlign: Receiver==null, because planName, teamMateTaskName does not match" << endl;
//        }
//        /*PROTECTED REGION END*/
//    }
///*PROTECTED REGION ID(methods1455888574532) ENABLED START*/ //Add additional methods here
///*PROTECTED REGION END*/
//} /* namespace alica */
