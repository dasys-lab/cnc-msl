using namespace std;
#include "Plans/GameStrategy/Other/DropBallAttackerPos.h"

/*PROTECTED REGION ID(inccpp1455537841488) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include <msl_robot/MSLRobot.h>
#include "container/CNPoint2D.h"
#include <Ball.h>
#include <RawSensorData.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
using namespace geometry;
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1455537841488) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DropBallAttackerPos::DropBallAttackerPos() :
            DomainBehaviour("DropBallAttackerPos")
    {
        /*PROTECTED REGION ID(con1455537841488) ENABLED START*/ //Add additional options here
        query = make_shared<msl::MovementQuery>();
        /*PROTECTED REGION END*/
    }
    DropBallAttackerPos::~DropBallAttackerPos()
    {
        /*PROTECTED REGION ID(dcon1455537841488) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DropBallAttackerPos::run(void* msg)
    {
        /*PROTECTED REGION ID(run1455537841488) ENABLED START*/ //Add additional options here

        auto ownPos = wm->rawSensorData->getOwnPositionVision();
        if (ownPos == nullptr)
        {
            cerr << "No own Position!!!! Initiating Selfdestruction !!!" << endl;
            return;
        }

        auto alloBallPos = wm->ball->getAlloBallPosition();
        if (alloBallPos == nullptr)
        {
        	cout << "DBAP: alo ball pos null" << endl;
            alloBallPos = make_shared < geometry::CNPoint2D > (0, 0);
        }



        auto alloTarget = wm->field->posOwnGoalMid() - alloBallPos;
        alloTarget = alloTarget->normalize() * 1250;
        alloTarget = alloBallPos + alloTarget;

        auto egoTarget = alloTarget->alloToEgo(*ownPos);
        auto egoAlignPoint = alloBallPos->alloToEgo(*ownPos);

        msl_actuator_msgs::MotionControl mc;
//        mc = msl::RobotMovement::moveToPointCarefully(egoTarget, egoAlignPoint, 100, nullptr);
        query->egoDestinationPoint = egoTarget;
        query->egoAlignPoint = egoAlignPoint;
        query->snapDistance = 100;
        mc = this->robot->robotMovement->moveToPoint(query);

        if (!std::isnan(mc.motion.translation))
        {
            send(mc);
        }
        else
        {
            cout << "Motion command is NaN!" << endl;
        }

        /*PROTECTED REGION END*/
    }
    void DropBallAttackerPos::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1455537841488) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1455537841488) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
