using namespace std;
#include "Plans/Standards/Own/Penalty/PenaltyPosExecuter.h"

/*PROTECTED REGION ID(inccpp1466940407563) ENABLED START*/ //Add additional includes here
#include <MSLFootballField.h>
#include <MSLWorldModel.h>
#include <container/CNPoint2D.h>
#include <msl_robot/robotmovement/MovementQuery.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <RawSensorData.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1466940407563) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    PenaltyPosExecuter::PenaltyPosExecuter() :
            DomainBehaviour("PenaltyPosExecuter")
    {
        /*PROTECTED REGION ID(con1466940407563) ENABLED START*/ //Add additional options here
        this->query = make_shared<msl::MovementQuery>();
        this->translation = (*this->sc)["Drive"]->get<double>("Drive", "Default", "Velocity", NULL);
        this->catchRadius = (*this->sc)["Drive"]->get<double>("Drive", "Default", "CatchRadius", NULL);
        this->alloTarget = this->wm->field->posOppPenaltyMarker();
        this->alloTarget->x -= 1000.0;
        /*PROTECTED REGION END*/
    }
    PenaltyPosExecuter::~PenaltyPosExecuter()
    {
        /*PROTECTED REGION ID(dcon1466940407563) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void PenaltyPosExecuter::run(void* msg)
    {
        /*PROTECTED REGION ID(run1466940407563) ENABLED START*/ //Add additional options here
        msl::RobotMovement rm;
        auto ownPos = wm->rawSensorData->getOwnPositionVision();
        if (!ownPos)
        {
            return;
        }
        auto egoTarget = alloTarget->alloToEgo(*ownPos);
        auto alloBall = wm->ball->getAlloBallPosition();
        if (alloBall != nullptr)
        {
            auto additionalPoints = make_shared<vector<shared_ptr<geometry::CNPoint2D>>>();
            additionalPoints->push_back(alloBall);
            query->additionalPoints = additionalPoints;
        }

        msl_actuator_msgs::MotionControl mc;
        query->egoDestinationPoint = egoTarget;
        query->egoAlignPoint = this->wm->field->posOppGoalMid()->alloToEgo(*ownPos);
        mc = rm.moveToPoint(query);

        if (egoTarget->length() < this->catchRadius)
        {
            this->setSuccess(true);
        }

        if (!std::isnan(mc.motion.translation))
        {
            send(mc);
        }
        /*PROTECTED REGION END*/
    }
    void PenaltyPosExecuter::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1466940407563) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1466940407563) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
