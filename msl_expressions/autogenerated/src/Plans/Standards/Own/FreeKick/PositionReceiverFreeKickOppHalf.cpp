using namespace std;
#include "Plans/Standards/Own/FreeKick/PositionReceiverFreeKickOppHalf.h"

/*PROTECTED REGION ID(inccpp1464780799716) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include "SystemConfig.h"
#include <RawSensorData.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1464780799716) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    PositionReceiverFreeKickOppHalf::PositionReceiverFreeKickOppHalf() :
            DomainBehaviour("PositionReceiverFreeKickOppHalf")
    {
        /*PROTECTED REGION ID(con1464780799716) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    PositionReceiverFreeKickOppHalf::~PositionReceiverFreeKickOppHalf()
    {
        /*PROTECTED REGION ID(dcon1464780799716) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void PositionReceiverFreeKickOppHalf::run(void* msg)
    {
        /*PROTECTED REGION ID(run1464780799716) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball->getEgoBallPosition();
        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }
        shared_ptr < geometry::CNPoint2D > alloBall = egoBallPos->egoToAllo(*ownPos);
        // Create additional points for path planning
        shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<
                vector<shared_ptr<geometry::CNPoint2D>>>();
        // add alloBall to path planning
        additionalPoints->push_back(alloBall);

        //the receiver should stand on a line with the middle of the goal with minimum allowed distance to ball in opp half

        auto alloGoalMid = this->wm->field->posOppGoalMid();

        auto lineVect = alloBall - alloGoalMid;

        alloTarget = alloBall + lineVect->normalize() * 2300;

//		alloTarget->y = alloBall->y;
//		alloTarget->x = alloBall->x - 2300;

        shared_ptr < geometry::CNPoint2D > egoTarget = alloTarget->alloToEgo(*ownPos);

        msl_actuator_msgs::MotionControl mc;

        // ask the path planner how to get there
        mc = msl::RobotMovement::moveToPointCarefully(egoTarget, egoBallPos, 0, additionalPoints);

        // if we reach the point and are aligned, the behavior is successful
        if (egoTarget->length() < 250 && fabs(egoBallPos->rotate(M_PI)->angleTo()) < (M_PI / 180) * 5)
        {
            this->setSuccess(true);
        }
        send(mc);
        /*PROTECTED REGION END*/
    }
    void PositionReceiverFreeKickOppHalf::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1464780799716) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1464780799716) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
