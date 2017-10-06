using namespace std;
#include "Plans/Standards/Own/StdAlignSingleRobot.h"

/*PROTECTED REGION ID(inccpp1467385758084) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/RobotMovement.h"
#include "msl_robot/robotmovement/MovementQuery.h"
#include <RawSensorData.h>
#include <Ball.h>
#include <Robots.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
using geometry::CNPointAllo;
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1467385758084) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    StdAlignSingleRobot::StdAlignSingleRobot() :
            DomainBehaviour("StdAlignSingleRobot")
    {
        /*PROTECTED REGION ID(con1467385758084) ENABLED START*/ //Add additional options here
        this->executorDistanceToBall = (*this->sc)["StandardSituation"]->get<double>("StandardAlignToPoint",
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
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        auto egoBallPos = wm->ball->getPositionEgo();

        // return if necessary information is missing
        if (!ownPos || !egoBallPos)
        {
            return;
        }

        // Create allo ball
        auto alloBall = egoBallPos->toAllo(*ownPos);

        // Create additional points for path planning
        auto additionalPoints = nonstd::make_optional<vector<geometry::CNPointAllo>>();
        // add alloBall to path planning
        additionalPoints->push_back(alloBall);

        auto aimPoint = CNPointAllo(wm->field->posLeftOppRestartMarker().x,
                                                    wm->field->posLeftOppRestartMarker().y + 1000);
        if (alloBall.y > 0)
        {
            aimPoint =CNPointAllo(wm->field->posRightOppRestartMarker().x,
                                                        wm->field->posRightOppRestartMarker().y - 1000);
        }

        auto egoTarget = (alloBall + (alloBall - aimPoint).normalize() * this->executorDistanceToBall).toEgo(*ownPos);
//		egoTarget = (alloBall + ((alloBall - receiverPos)->normalize() * this->executerDistanceToBall))->alloToEgo(
//				*ownPos);

        msl_actuator_msgs::MotionControl mc;
        msl::RobotMovement rm;
        this->m_Query.egoAlignPoint = egoBallPos;
        this->m_Query.additionalPoints = additionalPoints;
        this->m_Query.egoDestinationPoint = egoTarget;
        mc = rm.moveToPoint(m_Query);
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
