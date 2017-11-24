using namespace std;
#include "Plans/TestPlans/DribbleControlTest/DribbleToAttackPointTest.h"

/*PROTECTED REGION ID(inccpp1498664309837) ENABLED START*/ //Add additional includes here
#include <MSLWorldModel.h>
#include <RawSensorData.h>
#include <Ball.h>
#include <msl_actuator_msgs/MotionControl.h>
#include <msl_robot/robotmovement/RobotMovement.h>
//#include "msl_msgs/PathPlanner.h"
#include <pathplanner/VoronoiNet.h>
//#include "msl_msgs/VoronoiNetInfo.h"
#include "pathplanner/evaluator/PathEvaluator.h"
#include "pathplanner/PathProxy.h"
#include <pathplanner/PathPlanner.h>

using nonstd::optional;
using nonstd::nullopt;
using nonstd::make_optional;


/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1498664309837) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DribbleToAttackPointTest::DribbleToAttackPointTest() :
            DomainBehaviour("DribbleToAttackPointTest")
    {
        /*PROTECTED REGION ID(con1498664309837) ENABLED START*/ //Add additional options here
        sc = nullptr;
        wheelSpeed = 0;
        lastRotError = 0;
        ownPenalty = false;
        pastRotation =
        {};
        counter = 0;
        maxVel = 0;
        maxOppDist = 0;
        maxDribbleSpeed = 0;
        oppVectorWeight = 0;
        clausenDepth = 0;
        clausenPow = 0;
        pastRotationSize = 0;
        orthoDriveWeight = 0;
        targetDriveWeight = 0;

        /*PROTECTED REGION END*/
    }
    DribbleToAttackPointTest::~DribbleToAttackPointTest()
    {
        /*PROTECTED REGION ID(dcon1498664309837) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DribbleToAttackPointTest::run(void* msg)
    {
        /*PROTECTED REGION ID(run1498664309837) ENABLED START*/ //Add additional options here
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        auto egoBallPos = wm->ball->getPositionEgo();
        msl::RobotMovement rm;
        msl_actuator_msgs::MotionControl mc;
        auto vNet = wm->pathPlanner->getCurrentVoronoiNet();

        if (!ownPos)
        {
            cerr << "DribbleToAttackPointTest::run(): own position is nullptr!" << endl;
            return;
        }
        if (!egoBallPos)
        {
            cerr << "DribbleToAttackPointTest::run(): egoBallPos is nullptr!" << endl;
        }
        if (!vNet)
        {
            cerr << "DribbleToAttackPointTest::run(): voronoi net is nullptr!" << endl;
        }

        // drive to destination point

        //fill query
        auto egoDestinationPoint = destinationPoint.toEgo(*ownPos);
        query.egoDestinationPoint = nonstd::make_optional<geometry::CNPointEgo>(egoDestinationPoint);

        // get movement command
        mc = rm.moveToPoint(query);

        // turn the robot to the nearest opponent while he is rotating around the ball
        auto closestOpponent = getClosestOpp();
        optional < geometry::CNPointAllo > alloAlignPoint = nullopt;

        //if there is no oppen closer than maxOppDist
        if (!closestOpponent)
        {
//			query->egoAlignPoint = egoBallPos;
            cout << "no opponent nearby!" << endl;
            return;
        }
        else
        {
            // OwnPos = P1, closestOpp = P2
            // oppVec = P1 - P2 -> vector is pointing at ownPos
            auto oppVec = *ownPos - *closestOpponent;
            oppVec = oppVec * 1.2;
            // getting point out of vector
            alloAlignPoint = *closestOpponent + oppVec;
            cout << "============================" << endl;
//			cout << "ownPos:        x=" << ownPos->x << " y=" << ownPos->y << endl;
//			cout << "closesOpp:     x=" << closestOpponent->x << " y=" << closestOpponent->y << endl;
//			cout << "newVector:     x=" << oppVec->x << " y=" << oppVec->y << endl;
//			cout << "alloAlignPoint: x=" << alloAlignPoint->x << " y=" << alloAlignPoint->y << endl;

            // for testing
//			alloAlignPoint = closestOpponent;
            auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
            query.egoAlignPoint = alloAlignPoint->toEgo(*ownPos);
        }

        query.rotateAroundTheBall = true;
        mc = rm.alignTo(query);
        // use rm.alignTo() method
        // try to combine moveToPoint and alignTo

        // send movement command
        if (mc.motion.rotation == NAN || mc.motion.translation == NAN || mc.motion.angle == NAN)
        {
            cerr << "DribbleToAttackPoint::run(): Motion command is NaN!" << endl;
            return;
        }
        send(mc);
        /*PROTECTED REGION END*/
    }
    void DribbleToAttackPointTest::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1498664309837) ENABLED START*/ //Add additional options here
//		wheelSpeed = -80;
//		lastClosesOpp = nullptr;
//		lastRotError = 0;
//		for (int i = 0; i < pastRotation.size(); i++)
//		{
//			pastRotation.at(i) = 0;
//		}
//		counter = -1;
        readConfigParameters();
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1498664309837) ENABLED START*/ //Add additional methods here
    optional<geometry::CNPointAllo> DribbleToAttackPointTest::getClosestOpp()
    {
        //get opponents
        auto vNet = wm->pathPlanner->getCurrentVoronoiNet();
        auto opponents = vNet->getOpponentPositions();
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();

        // find nearest opponent
        nonstd::optional<geometry::CNPointAllo > closestOpponent = nullopt;
        double lowestDist = numeric_limits<double>::max();
        double dist = 0;

        for (int i = 0; i < opponents->size(); i++)
        {
            auto opp = opponents->at(i);
            auto ownPoint = ownPos->getPoint();
            dist = opp.distanceTo(ownPoint);
            if (dist < maxOppDist)
            {
                if (dist < lowestDist)
                {
                    lowestDist = dist;
                    closestOpponent = opp;
                }
            }
        }

        return closestOpponent;
    }

    void DribbleToAttackPointTest::readConfigParameters()
    {
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
        pastRotation.resize(pastRotationSize);
    }
/*PROTECTED REGION END*/
} /* namespace alica */
