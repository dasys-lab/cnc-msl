using namespace std;
#include "Plans/Example/MoveThaoCircleBack.h"

/*PROTECTED REGION ID(inccpp1450092434819) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1450092434819) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    MoveThaoCircleBack::MoveThaoCircleBack() :
            DomainBehaviour("MoveThaoCircleBack")
    {
        /*PROTECTED REGION ID(con1450092434819) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    MoveThaoCircleBack::~MoveThaoCircleBack()
    {
        /*PROTECTED REGION ID(dcon1450092434819) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void MoveThaoCircleBack::run(void* msg)
    {
        /*PROTECTED REGION ID(run1450092434819) ENABLED START*/ //Add additional options here
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData.getOwnPositionVision();
        if (ownPos == nullptr)
        {
            return;
        }
        geometry::CNPoint2D alloTarget;

        //   alloTarget.x = 1000;
        //   alloTarget.y = 10000;
        auto egoTarget = alloTarget.alloToEgo(*ownPos);

        msl_actuator_msgs::MotionControl mcon;
        mcon.motion.angle = 0;
        mcon.motion.rotation = -0.4;
        mcon.motion.translation = 1000;

        if (egoTarget->length() > 350)
        {
            this->haveBeenFarAway = true;
        }

        if (this->haveBeenFarAway && egoTarget->length() < 250)
        {
            this->success = true;
        }
        send(mcon);
        cout << "ownPos=" << (*ownPos).toString();
        /*PROTECTED REGION END*/
    }
    void MoveThaoCircleBack::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1450092434819) ENABLED START*/ //Add additional options here
        this->haveBeenFarAway = false;
        ownPos = wm->rawSensorData.getOwnPositionVision();

        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1450092434819) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
