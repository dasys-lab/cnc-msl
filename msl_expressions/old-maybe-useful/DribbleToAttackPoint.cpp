using namespace std;
#include "DribbleToAttackPoint.h"

/*PROTECTED REGION ID(inccpp1436855838589) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include "msl_msgs/PathPlanner.h"
#include "msl_msgs/VoronoiNetInfo.h"
#include "pathplanner/evaluator/PathEvaluator.h"
#include "pathplanner/PathProxy.h"
#include <Ball.h>
#include <RawSensorData.h>
#include <pathplanner/PathPlanner.h>
#include <msl_actuator_msgs/BallHandleCmd.h>
using geometry::CNPointEgo;
using geometry::CNPointAllo;
using geometry::CNVecEgo;
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1436855838589) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DribbleToAttackPoint::DribbleToAttackPoint() :
            DomainBehaviour("DribbleToAttackPoint")
    {
        /*PROTECTED REGION ID(con1436855838589) ENABLED START*/ //Add additional options here
        this->wheelSpeed = -50;
        this->sc = nullptr;
        this->maxVel = (*this->sc)["Dribble"]->get<double>("DribbleToAttackPoint.maxVel", NULL);
        this->maxOppDist = (*this->sc)["Dribble"]->get<double>("DribbleToAttackPoint.maxOppDist", NULL);
        this->oppVectorWeight = (*this->sc)["Dribble"]->get<double>("DribbleToAttackPoint.oppVectorWeight", NULL);
        this->clausenDepth = (*this->sc)["Dribble"]->get<int>("DribbleToAttackPoint.clausenDepth", NULL);
        this->clausenPow = (*this->sc)["Dribble"]->get<int>("DribbleToAttackPoint.clausenPow", NULL);
        this->pastRotationSize = (*this->sc)["Dribble"]->get<double>("DribbleToAttackPoint.pastRotationSize", NULL);
        this->orthoDriveWeight = (*this->sc)["Dribble"]->get<double>("DribbleToAttackPoint.orthoDriveWeight", NULL);
        this->targetDriveWeight = (*this->sc)["Dribble"]->get<double>("DribbleToAttackPoint.targetDriveWeight", NULL);
        this->maxDribbleSpeed = (*this->sc)["Dribble"]->get<double>("DribbleToAttackPoint.maxDribbleSpeed", NULL);
        voroniPub = n.advertise<msl_msgs::VoronoiNetInfo>("/PathPlanner/VoronoiNet", 10);
        pastRotation.resize(pastRotationSize);
        lastRotError = 0;
        ownPenalty = false;
        counter = -1;

        /*PROTECTED REGION END*/
    }
    DribbleToAttackPoint::~DribbleToAttackPoint()
    {
        /*PROTECTED REGION ID(dcon1436855838589) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DribbleToAttackPoint::run(void* msg)
    {
        /*PROTECTED REGION ID(run1436855838589) ENABLED START*/ //Add additional options here
        //get own Pos
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        //get voronoi net
        auto vNet = wm->pathPlanner->getCurrentVoronoiNet();
        //get ego bal pos
        auto egoBallPos = wm->ball->getPositionEgo();
        CNPointEgo egoAlignPoint;
        CNVecEgo alignPointVec;
        //check if need information is available
        if (!ownPos || !vNet || !egoBallPos)
        {
            cout << "returning" << endl;
            return;
        }
        //get opponents positions
        auto opponents = vNet->getOpponentPositions();

        //Constant ball handle wheel speed for testing
        msl_actuator_msgs::BallHandleCmd bhc;
        bhc.leftMotor = (int8_t)this->wheelSpeed;
        bhc.rightMotor = (int8_t)this->wheelSpeed;
        send(bhc);

        //get point of pos
        auto ownPoint = ownPos->getPoint();
        //claculate ego target
        egoTargetPoint = alloTargetPoint.toEgo(*ownPos);

        //find closes opponent in maxOppDist
        nonstd::optional<CNPointAllo> closestOpponent = nonstd::nullopt;
        double lowestDist = numeric_limits<double>::max();
        double dist = 0;
        for (int i = 0; i < opponents->size(); i++)
        {
            auto opp = opponents->at(i);
            dist = opp.distanceTo(ownPoint);
            if (dist < maxOppDist)
            {
                if (dist < lowestDist)
                {
                    lowestDist = dist;
                    closestOpponent = nonstd::make_optional<CNPointAllo>(opp);
                }
            }
        }
        //if there is no oppen closer than maxOppDist
        if (!closestOpponent)
        {
            egoAlignPoint = egoTargetPoint;
        }
        //if ther is an opponent close enough
        else
        {
            //change to ego coordinates
            auto closestOpponentEgo = closestOpponent->toEgo(*ownPos);
            //calculate weighted vector to ego opp
            auto weightedOppVector = closestOpponentEgo.rotateZ(M_PI) * (1.0 / closestOpponentEgo.length())
                    * (this->oppVectorWeight - closestOpponentEgo.length())/*egoTargetPoint->length() / 5.0*/;
            //calculate weighted vector to target point
            auto weightedTargetVector = egoTargetPoint * (1.0 / egoTargetPoint.length()) * closestOpponentEgo.length();
            //calculate align point facing away from opp
            alignPointVec = (weightedOppVector + weightedTargetVector).normalize() * 1000;
        }
        //debug msg
        msl_msgs::VoronoiNetInfo netMsg;
        if (closestOpponent)
        {
            msl_msgs::Point2dInfo info;
            info.x = closestOpponent->x;
            info.y = closestOpponent->y;
            netMsg.sites.push_back(info);

//			if (lastClosesOpp != nullptr && (closestOpponent - lastClosesOpp)->length() > 1000)
//			{
//				cout << "changed last closest opp" << endl;
//			}
            lastClosestOpp = closestOpponent;
        }
        //create motion control
        msl_actuator_msgs::MotionControl mc;

        //get way from path planner
        auto temp = msl::PathProxy::getInstance()->getEgoDirection(egoTargetPoint, *eval);
        //if angle to alignpoint is too high
        if (alignPointVec.rotateZ(M_PI).angleZ() > M_PI / 2)
        {
            mc.motion.rotation = 2 * M_PI;
        }
        else if (alignPointVec.rotateZ(M_PI).angleZ() < -M_PI / 2)
        {
            mc.motion.rotation = -2 * M_PI;
        }
        else
        {
            //adjustment
//        	sinus regelung
//			mc.motion.rotation = egoAlignPoint->rotate(M_PI)->angleTo()
//                    * abs(sin(egoAlignPoint->rotate(M_PI)->angleTo())) * 2; // + (egoAlignPoint->rotate(M_PI)->angleTo() - lastRotError) * 0.3;
//			wurzel regelung
//        	mc.motion.rotation = egoAlignPoint->rotate(M_PI)->angleTo()
//					* sqrt(abs(egoAlignPoint->rotate(M_PI)->angleTo())) * 2;
            //calculate clausen function
            double clausenValue = 0.0;
            for (int i = 1; i < this->clausenDepth; i++)
            {
                clausenValue += sin(i * alignPointVec.rotateZ(M_PI).angleZ()) / pow(i, this->clausenPow);
            }
            // calculate past rotation average
            double sum = 0;
            for (int i = 0; i < pastRotation.size(); i++)
            {
                sum += pastRotation.at(i);
            }
            //adjust rotation
            mc.motion.rotation = (sum + alignPointVec.rotateZ(M_PI).angleZ() * abs(clausenValue))
                    / (this->pastRotationSize + 1); // *4
            counter++;
            //set past rotation
            pastRotation.at(counter % this->pastRotationSize) = alignPointVec.rotateZ(M_PI).angleZ()
                    * abs(clausenValue);
        }
        // crate the motion orthogonal to the ball
        auto driveTo = egoBallPos->rotateZ(-M_PI / 2.0);
        //TODO create 2 parameter
        double rotationWeight = mc.motion.rotation * this->orthoDriveWeight;
        driveTo = driveTo * rotationWeight;

        // add the motion towards the ball
        driveTo = driveTo
                + temp->normalize() * min(maxDribbleSpeed, temp->length())
                        / (1 + abs(mc.motion.rotation) * this->targetDriveWeight);

        mc.motion.angle = driveTo.angleZ();
        mc.motion.translation = min(this->maxVel, driveTo.length());
//		cout << "Rotation " << mc.motion.rotation << " Angle " << egoAlignPoint->rotate(M_PI)->angleTo() << endl;
        //debug msg
        msl_msgs::Point2dInfo info;
        info.x = egoAlignPoint.toAllo(*ownPos).x;
        info.y = egoAlignPoint.toAllo(*ownPos).y;
        netMsg.sites.push_back(info);
        //check if goal is reached
        if (egoTargetPoint.length() < 250)
        {
            this->setSuccess(true);
        }
        //save last error
        lastRotError = alignPointVec.rotateZ(M_PI).angleZ();

        // TODO von Taker: Am Arsch, richtige nachricht verwenden, dann klappts auch mit der Anzeige!!!!
//        voroniPub.publish(netMsg);

        //send motion control
        send(mc);
        /*PROTECTED REGION END*/
    }
    void DribbleToAttackPoint::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1436855838589) ENABLED START*/ //Add additional options here
        sc = supplementary::SystemConfig::getInstance();
        eval = make_shared<msl::PathEvaluator>();
        bool success = true;
        string tmp = "";
        success &= getParameter("OwnPenalty", tmp);
        try
        {
            if (success)
            {
                std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::tolower);
                std::istringstream is(tmp);
                bool b;
                is >> std::boolalpha >> b;
                ownPenalty = b;
            }

        }
        catch (exception& e)
        {
            cerr << "Could not cast the parameter properly" << endl;
        }
        if (!success)
        {
            cerr << "D2AP: Parameter does not exist" << endl;
        }
        if (!ownPenalty)
        {
            alloTargetPoint = wm->field->posOppPenaltyMarker();
        }
        else
        {
            alloTargetPoint = wm->field->posOwnPenaltyMarker();
        }
        wheelSpeed = -80;
        lastRotError = 0;
        for (int i = 0; i < pastRotation.size(); i++)
        {
            pastRotation.at(i) = 0;
        }
        counter = -1;
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1436855838589) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
