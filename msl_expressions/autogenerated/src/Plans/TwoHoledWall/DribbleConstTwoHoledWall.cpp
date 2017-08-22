using namespace std;
#include "Plans/TwoHoledWall/DribbleConstTwoHoledWall.h"

/*PROTECTED REGION ID(inccpp1496840140891) ENABLED START*/ //Add additional includes here
#include <SystemConfig.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1496840140891) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DribbleConstTwoHoledWall::DribbleConstTwoHoledWall() :
            DomainBehaviour("DribbleConstTwoHoledWall")
    {
        /*PROTECTED REGION ID(con1496840140891) ENABLED START*/ //Add additional options here
        this->wheelSpeed = (*this->sc)["Show"]->get<int>("TwoHoledWall.WheelSpeed", NULL);
        /*PROTECTED REGION END*/
    }
    DribbleConstTwoHoledWall::~DribbleConstTwoHoledWall()
    {
        /*PROTECTED REGION ID(dcon1496840140891) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DribbleConstTwoHoledWall::run(void* msg)
    {
        /*PROTECTED REGION ID(run1496840140891) ENABLED START*/ //Add additional options here
        // Constant ball handle wheel speed
        msl_actuator_msgs::BallHandleCmd bhc;
        bhc.leftMotor = this->wheelSpeed;
        bhc.rightMotor = this->wheelSpeed;
        send(bhc);
        /*PROTECTED REGION END*/
    }
    void DribbleConstTwoHoledWall::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1496840140891) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1496840140891) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
