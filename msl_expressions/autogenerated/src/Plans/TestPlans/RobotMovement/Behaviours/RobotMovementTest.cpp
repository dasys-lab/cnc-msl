using namespace std;
#include "Plans/TestPlans/RobotMovement/Behaviours/RobotMovementTest.h"

/*PROTECTED REGION ID(inccpp1473862842303) ENABLED START*/ //Add additional includes here
#include <Game.h>
#include "msl_robot/robotmovement/RobotMovement.h"
#include <RawSensorData.h>
#include <pathplanner/PathPlanner.h>
#include <MSLWorldModel.h>
#include <container/CNPoint2D.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1473862842303) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    RobotMovementTest::RobotMovementTest() :
            DomainBehaviour("RobotMovementTest")
    {
        /*PROTECTED REGION ID(con1473862842303) ENABLED START*/ //Add additional options here
        toX = 0;
        toY = 0;
        hadBall = false;
        /*PROTECTED REGION END*/
    }
    RobotMovementTest::~RobotMovementTest()
    {
        /*PROTECTED REGION ID(dcon1473862842303) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void RobotMovementTest::run(void* msg)
    {
        /*PROTECTED REGION ID(run1473862842303) ENABLED START*/ //Add additional options here
        bool testFirst = false;
        bool testSecond = true;

        if (testFirst)
        {
            auto ownPos = wm->rawSensorData->getOwnPositionVision();
            if (ownPos == nullptr)
                return;

            auto alloGoal = make_shared < geometry::CNPoint2D > (toX, toY);
            auto egoGoal = alloGoal->alloToEgo(*ownPos);
            auto egoBallPos = wm->ball->getEgoBallPosition();

            MotionControl mc;
            msl::RobotMovement rm;

//        cout << "haveBall = " << (wm->ball->haveBall() == true ? "true" : "false") << endl;
//        cout << "hadBall = " << (hadBall == true ? "true" : "false") << endl;
            if (!wm->ball->haveBall() && !hadBall)
            {
//        	cout << "getBall" << endl;
                query->egoDestinationPoint = egoBallPos;
                query->egoAlignPoint = egoBallPos;
                mc = rm.moveToPoint(query);
                send(mc);
                return;
            }

//        cout << "try to drive to " << toX << "|" << toY << endl;
            hadBall = true;
            query->egoDestinationPoint = egoGoal;
            query->egoAlignPoint = egoGoal;

            auto motionCommand = rm.moveToPoint(query);
            this->send(motionCommand);

            if (egoGoal->length() < 300)
            {
                this->setSuccess(true);
            }
        }
        else if (testSecond)
        {
            auto egoBallPos = wm->ball->getEgoBallPosition();
            auto alloGoalPos = wm->field->posOwnGoalMid();
            auto egoGoalPos = alloGoalPos->alloToEgo(*wm->rawSensorData->getOwnPositionVision());
            msl::RobotMovement rm;
            MotionControl mc;
            if (wm->ball->haveBall())
            {

                if (egoBallPos == nullptr)
                    return;

                query->egoAlignPoint = egoGoalPos;
                query->rotateAroundTheBall = true;
                double angleInDegree = 10;
                query->angleTolerance = (2 * M_PI) / 360 * angleInDegree;
                mc = rm.alignTo(query);
            }
            else
            {
                query->egoDestinationPoint = egoBallPos;
                query->egoAlignPoint = egoBallPos;
                mc = rm.moveToPoint(query);
            }
//            cout << "angle to goal = " << egoGoalPos->angleTo() << endl;
            cout << "Angle = " << mc.motion.angle << " Trans = " << mc.motion.translation << " Rot = "
                    << mc.motion.rotation << endl;
            send(mc);

        }

        /*PROTECTED REGION END*/
    }
    void RobotMovementTest::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1473862842303) ENABLED START*/ //Add additional options here
        query = make_shared<msl::MovementQuery>();
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1473862842303) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
