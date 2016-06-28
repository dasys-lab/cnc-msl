using namespace std;
#include "Plans/Standards/Own/FreeKick/AlignFreeGoalSpace.h"

/*PROTECTED REGION ID(inccpp1467039782450) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/MovementQuery.h>
#include "msl_robot/robotmovement/RobotMovement.h"
#include "SystemConfig.h"
#include <RawSensorData.h>
#include <Robots.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
#include <obstaclehandler/Obstacles.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1467039782450) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    AlignFreeGoalSpace::AlignFreeGoalSpace() :
            DomainBehaviour("AlignFreeGoalSpace")
    {
        /*PROTECTED REGION ID(con1467039782450) ENABLED START*/ //Add additional options here
        query = make_shared<msl::MovementQuery>();
        /*PROTECTED REGION END*/
    }
    AlignFreeGoalSpace::~AlignFreeGoalSpace()
    {
        /*PROTECTED REGION ID(dcon1467039782450) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void AlignFreeGoalSpace::run(void* msg)
    {
        /*PROTECTED REGION ID(run1467039782450) ENABLED START*/ //Add additional options here
        msl::RobotMovement rm;
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball->getEgoBallPosition();

        auto opps = wm->robots->opponents.getOpponentsAlloClustered();

        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }
        shared_ptr < geometry::CNPoint2D > alloBall = egoBallPos->egoToAllo(*ownPos);
        shared_ptr < geometry::CNPoint2D > alloAlignPoint = nullptr;
        shared_ptr < geometry::CNPoint2D > egoAlignPoint = nullptr;

        // try shooting in the long side of the goal
        if (alloBall->y > wm->field->posLeftOppGoalPost()->y)
        {
            alloAlignPoint = make_shared < geometry::CNPoint2D
                    > (wm->field->posLeftOppGoalPost()->x, wm->field->posRightOppGoalPost()->y + 500);
        }
        else if (alloBall->y < wm->field->posRightOppGoalPost()->y)
        {
            alloAlignPoint = make_shared < geometry::CNPoint2D
                    > (wm->field->posRightOppGoalPost()->x, wm->field->posLeftOppGoalPost()->y - 500);
        }
        else // standing in the middle so get biggest free area
        {
            if (wm->obstacles->getBiggestFreeGoalAreaMidPoint() != nullptr)
            {

                egoAlignPoint =
                        make_shared < geometry::CNPoint2D
                                > (wm->obstacles->getBiggestFreeGoalAreaMidPoint()->x, wm->obstacles->getBiggestFreeGoalAreaMidPoint()->y);
            }
        }

        if (alloAlignPoint == nullptr && egoAlignPoint == nullptr)
        {
            egoAlignPoint = egoBallPos;
            cout << egoAlignPoint->y << endl;
        }
        else if (alloAlignPoint != nullptr)
        {
            egoAlignPoint = alloAlignPoint->alloToEgo(*ownPos);
            cout << egoAlignPoint->y << endl;
        } //else alignpoint should be set

        msl_actuator_msgs::MotionControl mc;
        query->egoDestinationPoint = egoAlignPoint;
        query->egoAlignPoint = egoAlignPoint;
        query->additionalPoints = nullptr;
        mc = rm.moveToPoint(query);

        if (!std::isnan(mc.motion.translation))
        {
            mc.motion.translation = 0;
            send(mc);
        }
        /*PROTECTED REGION END*/
    }
    void AlignFreeGoalSpace::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1467039782450) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1467039782450) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
