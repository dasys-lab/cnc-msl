using namespace std;
#include "Plans/Standards/Own/StdAlignSingleRobot.h"

/*PROTECTED REGION ID(inccpp1467385758084) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/RobotMovement.h>
#include <msl_robot/robotmovement/MovementQuery.h>
#include <msl_robot/MSLRobot.h>
#include <RawSensorData.h>
#include <Ball.h>
#include <Robots.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1467385758084) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    StdAlignSingleRobot::StdAlignSingleRobot() :
            DomainBehaviour("StdAlignSingleRobot")
    {
        /*PROTECTED REGION ID(con1467385758084) ENABLED START*/ //Add additional options here
        this->executerDistanceToBall = (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint",
                                                                                     "executerDistanceToBall", NULL);
        /*PROTECTED REGION END*/
    }
    StdAlignSingleRobot::~StdAlignSingleRobot()
    {
        /*PROTECTED REGION ID(dcon1467385758084) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void StdAlignSingleRobot::run(void* msg)
    {
        /*PROTECTED REGION ID(run1467385758084) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData->getOwnPositionVision();
        shared_ptr < geometry::CNPoint2D > egoBallPos = wm->ball->getEgoBallPosition();

        // return if necessary information is missing
        if (ownPos == nullptr || egoBallPos == nullptr)
        {
            return;
        }

        // Create allo ball
        shared_ptr < geometry::CNPoint2D > alloBall = egoBallPos->egoToAllo(*ownPos);

        // Create additional points for path planning
        auto additionalPoints = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
        // add alloBall to path planning
        additionalPoints->push_back(alloBall);

        shared_ptr < geometry::CNPoint2D > aimPoint = nullptr;
        aimPoint = make_shared < geometry::CNPoint2D
                > (wm->field->posLeftOppRestartMarker()->x, wm->field->posLeftOppRestartMarker()->y + 1000);
        if (alloBall->y > 0)
        {
            aimPoint = make_shared < geometry::CNPoint2D
                    > (wm->field->posRightOppRestartMarker()->x, wm->field->posRightOppRestartMarker()->y - 1000);
        }
        else
        {

        }
        shared_ptr < geometry::CNPoint2D > egoTarget;
        egoTarget = (alloBall + (alloBall - aimPoint)->normalize() * this->executerDistanceToBall)->alloToEgo(*ownPos);
//		egoTarget = (alloBall + ((alloBall - receiverPos)->normalize() * this->executerDistanceToBall))->alloToEgo(
//				*ownPos);

        msl_actuator_msgs::MotionControl mc;
        this->m_Query->egoAlignPoint = egoBallPos;
        this->m_Query->additionalPoints = additionalPoints;
        this->m_Query->egoDestinationPoint = egoTarget;
        mc = this->robot->robotMovement->moveToPoint(m_Query);
        send(mc);
        /*PROTECTED REGION END*/
    }
    void StdAlignSingleRobot::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1467385758084) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1467385758084) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
