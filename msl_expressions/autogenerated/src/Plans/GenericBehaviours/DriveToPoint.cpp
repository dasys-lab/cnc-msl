using namespace std;
#include "Plans/GenericBehaviours/DriveToPoint.h"

/*PROTECTED REGION ID(inccpp1417620568675) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1417620568675) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DriveToPoint::DriveToPoint() :
            DomainBehaviour("DriveToPoint")
    {
        /*PROTECTED REGION ID(con1417620568675) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    DriveToPoint::~DriveToPoint()
    {
        /*PROTECTED REGION ID(dcon1417620568675) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DriveToPoint::run(void* msg)
    {
        /*PROTECTED REGION ID(run1417620568675) ENABLED START*/ //Add additional options here
        auto me = wm->getOwnPosition();
        if (!me.operator bool())
        {
            return;
        }
        auto egoTarget = alloTarget.alloToEgo(*me);

        msl_actuator_msgs::MotionControl mc = RobotMovement::moveToPointCarefully(egoTarget, egoTarget, 150);

        if (egoTarget->length() < 250)
        {
            this->success = true;
        }

        send(mc);

        /*PROTECTED REGION END*/
    }
    void DriveToPoint::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1417620568675) ENABLED START*/ //Add additional options here
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
        }
        catch (exception& e)
        {
            cerr << "Could not cast the parameter properly" << endl;
        }
        if (!success)
        {
            cerr << "Parameter does not exist" << endl;
        }
        defaultTranslation = (*this->sc)["Drive"]->get<double>("Drive", "DefaultVelocity", NULL);
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1417620568675) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
