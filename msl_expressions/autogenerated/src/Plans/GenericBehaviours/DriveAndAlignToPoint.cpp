using namespace std;
#include "Plans/GenericBehaviours/DriveAndAlignToPoint.h"

/*PROTECTED REGION ID(inccpp1516808796983) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/RobotMovement.h>
#include <msl_robot/MSLRobot.h>
#include <RawSensorData.h>
#include <MSLWorldModel.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1516808796983) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DriveAndAlignToPoint::DriveAndAlignToPoint() :
            DomainBehaviour("DriveAndAlignToPoint")
    {
        /*PROTECTED REGION ID(con1516808796983) ENABLED START*/ //Add additional options here
        query = make_shared<msl::MovementQuery>();
        this->defaultTranslation = 1000;
        this->catchRadius = 250;
        /*PROTECTED REGION END*/
    }
    DriveAndAlignToPoint::~DriveAndAlignToPoint()
    {
        /*PROTECTED REGION ID(dcon1516808796983) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DriveAndAlignToPoint::run(void* msg)
    {
        /*PROTECTED REGION ID(run1516808796983) ENABLED START*/ //Add additional options here
        auto me = wm->rawSensorData->getOwnPositionVision();

        if (me == nullptr)
        {
            return;
        }
        auto egoTarget = alloTarget.alloToEgo(*me);
        auto egoOrientationTarget = alloOrientationTarget.alloToEgo(*me);

        msl_actuator_msgs::MotionControl mc;

        query->egoDestinationPoint = egoTarget;
        query->egoAlignPoint = egoOrientationTarget;
        query->snapDistance = catchRadius;
        //cout << "DriveAndAlignToPoint::run::query::Target " << query->egoDestinationPoint->x << " "
        //<< query->egoDestinationPoint->y << " " << query->egoDestinationPoint->z <<endl;
        //cout << "DriveAndAlignToPoint::run::query::Align " << query->egoAlignPoint->x << " " << query->egoAlignPoint->y
        // << " " << query->egoAlignPoint->z<<endl;
        mc = this->robot->robotMovement->moveToPoint(query);

        if (egoTarget->length() < catchRadius)
        {
            //mc.motion.translation = 0;
            //send(mc);
            //sleep(1);
            //cout << "DriveAndAlignToPoint: Success" << endl;
            this->setSuccess(true);
        }

        if (!std::isnan(mc.motion.translation))
        {
            send(mc);
        }

        /*PROTECTED REGION END*/
    }
    void DriveAndAlignToPoint::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1516808796983) ENABLED START*/ //Add additional options here
        string tmp;
        bool success = true;
        success &= getParameter("X", tmp);
        try
        {
            if (success)
            {
                alloTarget.x = stod(tmp);
            }
            success &= getParameter("Y", tmp);
            if (success)
            {
                alloTarget.y = stod(tmp);
            }
            success &= getParameter("OriX", tmp);
            if (success)
            {
                alloOrientationTarget.x = stod(tmp);
            }
            success &= getParameter("OriY", tmp);
            if (success)
            {
                alloOrientationTarget.y = stod(tmp);
            }
            success &= getParameter("R", tmp);
            if (success)
            {
                catchRadius = stod(tmp);
                catchRadius = max(catchRadius, 250.0);
            }
        }
        catch (exception& e)
        {
            cerr << "Could not cast the parameter properly" << endl;
        }
        if (!success)
        {
            cerr << "D2P: Parameter does not exist" << endl;
        }
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1516808796983) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
