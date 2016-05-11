using namespace std;
#include "Plans/GameStrategy/Other/DropBallAttackerPos.h"

/*PROTECTED REGION ID(inccpp1455537841488) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include "container/CNPoint2D.h"

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
        auto alloBallPos = wm->ball->getAlloBallPosition();
        if (alloBallPos == nullptr)
        {
            alloBallPos = make_shared < geometry::CNPoint2D > (0, 0);
        }

        auto ownPos = wm->rawSensorData->getOwnPositionVision();
        if (ownPos == nullptr)
        {
            cerr << "No own Position!!!! Initiating Selfdestruction !!!" << endl;
            return;
        }

        auto alloTarget = wm->field->posOwnGoalMid() - alloBallPos;
        alloTarget = alloTarget->normalize() * 1250;
        alloTarget = alloBallPos + alloTarget;

        auto egoTarget = alloTarget->alloToEgo(*ownPos);
        auto egoAlignPoint = alloBallPos->alloToEgo(*ownPos);

        msl_actuator_msgs::MotionControl mc;
        mc = msl::RobotMovement::moveToPointCarefully(egoTarget, egoAlignPoint, 100, nullptr);
        send(mc);

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
