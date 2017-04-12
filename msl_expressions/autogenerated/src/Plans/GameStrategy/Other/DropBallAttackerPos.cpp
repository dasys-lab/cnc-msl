using namespace std;
#include "Plans/GameStrategy/Other/DropBallAttackerPos.h"

/*PROTECTED REGION ID(inccpp1455537841488) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/RobotMovement.h>
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
        auto alloBallPos = wm->ball->getPositionAllo();
        if (!alloBallPos)
        {
            alloBallPos = geometry::CNPointAllo(0, 0);
        }

        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        if (!ownPos)
        {
            cerr << "No own Position!!!! Initiating Self-destruction !!!" << endl;
            return;
        }

        auto alloTargetVec = wm->field->posOwnGoalMid() - *alloBallPos;
        alloTargetVec = alloTargetVec.normalize() * 1250;
        auto alloTarget = *alloBallPos + alloTargetVec;

        auto egoTarget = alloTarget.toEgo(*ownPos);
        auto egoAlignPoint = alloBallPos->toEgo(*ownPos);

        msl_actuator_msgs::MotionControl mc;
//        mc = msl::RobotMovement::moveToPointCarefully(egoTarget, egoAlignPoint, 100, nullptr);
        query->egoDestinationPoint = egoTarget;
        query->egoAlignPoint = egoAlignPoint;
        query->snapDistance = 100;

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
