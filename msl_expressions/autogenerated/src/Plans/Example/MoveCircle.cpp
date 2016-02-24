using namespace std;
#include "Plans/Example/MoveCircle.h"

/*PROTECTED REGION ID(inccpp1449767365870) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1449767365870) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    MoveCircle::MoveCircle() :
            DomainBehaviour("MoveCircle")
    {
        /*PROTECTED REGION ID(con1449767365870) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    MoveCircle::~MoveCircle()
    {
        /*PROTECTED REGION ID(dcon1449767365870) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void MoveCircle::run(void* msg)
    {
        /*PROTECTED REGION ID(run1449767365870) ENABLED START*/ //Add additional options here
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
        mcon.motion.angle = M_PI;
        mcon.motion.rotation = 0.4;
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
        /*PROTECTED REGION END*/
    }
    void MoveCircle::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1449767365870) ENABLED START*/ //Add additional options here
        this->haveBeenFarAway = false;
        ownPos = wm->rawSensorData.getOwnPositionVision();
        // cout << "ownPos=" << ownPos->toString() << endl;
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1449767365870) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
