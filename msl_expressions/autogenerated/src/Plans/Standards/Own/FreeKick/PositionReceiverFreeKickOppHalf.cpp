using namespace std;
#include "Plans/Standards/Own/FreeKick/PositionReceiverFreeKickOppHalf.h"

/*PROTECTED REGION ID(inccpp1464780799716) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include "SystemConfig.h"
#include <RawSensorData.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
#include <obstaclehandler/Obstacles.h>
using nonstd::optional;
using nonstd::make_optional;
using geometry::CNPointAllo;
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
        msl::RobotMovement rm;
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        auto egoBallPos = wm->ball->getPositionEgo();
        if (!ownPos || !egoBallPos)
        {
            return;
        }

        auto alloBall = egoBallPos->toAllo(*ownPos);
        // Create additional points for path planning
        auto additionalPoints = make_optional<vector<CNPointAllo>>();
        // add alloBall to path planning
        additionalPoints->push_back(alloBall);

        optional<CNPointAllo> alloAlignPoint = nonstd::nullopt;
//		shared_ptr<geometry::CNPoint2D> egoAlignPoint = nullptr;

        //the receiver should stand on a line with the middle of the goal with minimum allowed distance to ball in opp half

//        shared_ptr < geometry::CNPoint2D > alloGoalMid = wm->field->posOppGoalMid();
//        shared_ptr < geometry::CNPoint2D > lineVect = alloBall - alloGoalMid;

//		alloTarget->y = alloBall->y;
//		alloTarget->x = alloBall->x - 2300;

        if (alloBall.y > 0)
        {
            alloAlignPoint = make_optional<CNPointAllo>(
                    wm->field->getFieldLength() / 2 + wm->ball->getBallDiameter(),
                    wm->field->posRightOppGoalPost().y + 450);
        } // align right-ish first to turn left later
        else
        {
            alloAlignPoint = make_optional<CNPointAllo>(
                    wm->field->getFieldLength() / 2 + wm->ball->getBallDiameter(),
                    wm->field->posLeftOppGoalPost().y - 450);
        }
//        else // standing in the middle so get biggest free area
//        {
//            if (wm->obstacles->getBiggestFreeGoalAreaMidPoint() != nullptr)
//            {
//                egoAlignPoint =
//                        make_shared < geometry::CNPoint2D
//                                > (wm->obstacles->getBiggestFreeGoalAreaMidPoint()->x, wm->obstacles->getBiggestFreeGoalAreaMidPoint()->y);
//            }
//
//        }

//        egoAlignPoint = wm->obstacles->getBiggestFreeGoalAreaMidPoint();
//        if (!egoAlignPoint)
//        {
//            egoAlignPoint = egoBallPos;
//        }

        //neither set
//        if (!alloAlignPoint && !egoAlignPoint)
//        {
//            egoAlignPoint = egoBallPos;
//        } // only allo
//        else if (alloAlignPoint)
//        {
////			alloTarget = alloBall + alloAlignPoint->normalize() * 2300;
//            egoAlignPoint = alloAlignPoint->alloToEgo(*ownPos);
//        } // else egoalignpoint should be set

        auto lineVect = alloBall - *alloAlignPoint;

        alloTarget = alloBall + lineVect.normalize() * 2300;
        alloTarget = this->wm->field->mapInsideField(*alloTarget);
        auto egoTarget = alloTarget->toEgo(*ownPos);
        msl_actuator_msgs::MotionControl mc;
        query.egoDestinationPoint = egoTarget;
        query.egoAlignPoint = alloAlignPoint->toEgo(*ownPos);
        query.additionalPoints = additionalPoints;
        mc = rm.moveToPoint(query);

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
    void PositionReceiverFreeKickOppHalf::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1464780799716) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1464780799716) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
