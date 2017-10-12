#include "Plans/Standards/Own/Penalty/InGame/Pos4PenaltyRebounce.h"

/*PROTECTED REGION ID(inccpp1466972686566) ENABLED START*/ //Add additional includes here
#include <MSLFootballField.h>
#include <MSLWorldModel.h>
#include <msl_robot/robotmovement/MovementQuery.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <RawSensorData.h>
#include <Rules.h>
#include <cnc_geometry/CNPositionAllo.h>
#include <cnc_geometry/CNPositionEgo.h>
#include <cnc_geometry/CNPointAllo.h>
#include <cnc_geometry/CNPointEgo.h>
#include <nonstd/optional.hpp>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1466972686566) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    Pos4PenaltyRebounce::Pos4PenaltyRebounce() :
            DomainBehaviour("Pos4PenaltyRebounce")
    {
        /*PROTECTED REGION ID(con1466972686566) ENABLED START*/ //Add additional options here
        this->translation = (*this->sc)["Drive"]->get<double>("Drive", "Default", "Velocity", NULL);
        this->catchRadius = (*this->sc)["Drive"]->get<double>("Drive", "Default", "CatchRadius", NULL);
        /*PROTECTED REGION END*/
    }
    Pos4PenaltyRebounce::~Pos4PenaltyRebounce()
    {
        /*PROTECTED REGION ID(dcon1466972686566) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void Pos4PenaltyRebounce::run(void* msg)
    {
        /*PROTECTED REGION ID(run1466972686566) ENABLED START*/ //Add additional options here
        msl::RobotMovement rm;
        auto ownPos = wm->rawSensorData->getOwnPositionVisionBuffer().getLastValidContent();
        if (!ownPos)
        {
            return;
        }
        auto egoTarget = alloTarget.toEgo(*ownPos);

        query.egoDestinationPoint = nonstd::make_optional<geometry::CNPointEgo>(egoTarget);
        query.egoAlignPoint = nonstd::make_optional<geometry::CNPointEgo>(
                this->wm->field->posOppGoalMid().toEgo(*ownPos));
        query.snapDistance = this->catchRadius;
        auto mc = rm.moveToPoint(query);

        if (egoTarget.length() < this->catchRadius)
        {
            this->setSuccess(true);
        }

        if (!std::isnan(mc.motion.translation))
        {
            send(mc);
        }
        /*PROTECTED REGION END*/
    }
    void Pos4PenaltyRebounce::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1466972686566) ENABLED START*/ //Add additional options here
        auto rules = msl::Rules::getInstance();
        auto result = this->parameters->find("OwnPenalty");
        if (result != this->parameters->end())
        {
            if (result->second.compare("true") == 0)
            {
                this->alloTarget = this->wm->field->posOwnPenaltyMarker();
            }
            else
            {
                this->alloTarget = this->wm->field->posOppPenaltyMarker();
            }
        }
        this->alloTarget.y += rules->getStayAwayRadiusOpp();

        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1466972686566) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
