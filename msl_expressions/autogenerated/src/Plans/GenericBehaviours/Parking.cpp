using namespace std;
#include "Plans/GenericBehaviours/Parking.h"

/*PROTECTED REGION ID(inccpp1429111623710) ENABLED START*/ //Add additional includes here
#include <SystemConfig.h>
#include <container/CNPoint2D.h>
#include <msl_robot/robotmovement/MovementQuery.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <msl_robot/MSLRobot.h>
#include <RawSensorData.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1429111623710) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    Parking::Parking() :
            DomainBehaviour("Parking")
    {
        /*PROTECTED REGION ID(con1429111623710) ENABLED START*/ //Add additional options here
        this->sc = supplementary::SystemConfig::getInstance();
        this->offset = (*this->sc)["Parking"]->get<double>("ParkingPositions", "offset", NULL);
        this->parkingSlotIdx = 0; // must be set in initialise parameters because of the availability of ownId;
        this->distanceToParkingPositionTolerance = (*this->sc)["Parking"]->get<double>(
                "ParkingPositions", "distanceToParkingPositionTolerance", NULL);
        this->movementQuery = make_shared<msl::MovementQuery>();
        this->parkingPosition = make_shared < geometry::CNPoint2D
                > (this->parkingSlotIdx * -this->offset, wm->field->getFieldWidth() / 2.0);
        /*PROTECTED REGION END*/
    }
    Parking::~Parking()
    {
        /*PROTECTED REGION ID(dcon1429111623710) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void Parking::run(void* msg)
    {
        /*PROTECTED REGION ID(run1429111623710) ENABLED START*/ //Add additional options here
        auto ownPos = wm->rawSensorData->getOwnPositionVision();
        if (ownPos == nullptr)
        {
            return;
        }
        if (ownPos->distanceTo(this->parkingPosition) < this->distanceToParkingPositionTolerance)
        {
            this->setSuccess(true);
            return;
        }

        this->movementQuery->egoDestinationPoint = this->parkingPosition->alloToEgo(*ownPos);
        this->movementQuery->egoAlignPoint =
                (this->parkingPosition + make_shared < geometry::CNPoint2D > (0, -1000.0))->alloToEgo(*ownPos);
        msl_actuator_msgs::MotionControl mc = this->robot->robotMovement->moveToPoint(movementQuery);
        send(mc);
        /*PROTECTED REGION END*/
    }
    void Parking::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1429111623710) ENABLED START*/ //Add additional options here
        this->parkingSlotIdx = (*this->sc)["Parking"]->get<double>("ParkingPositions",
                                                                   to_string(this->getOwnId()).c_str(), NULL);
        this->parkingPosition = make_shared < geometry::CNPoint2D
                > (this->parkingSlotIdx * -this->offset, wm->field->getFieldWidth() / 2.0);
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1429111623710) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
