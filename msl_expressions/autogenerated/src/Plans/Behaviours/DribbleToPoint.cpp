using namespace std;
#include "Plans/Behaviours/DribbleToPoint.h"

/*PROTECTED REGION ID(inccpp1414752367688) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1414752367688) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    DribbleToPoint::DribbleToPoint() :
            DomainBehaviour("DribbleToPoint")
    {
        /*PROTECTED REGION ID(con1414752367688) ENABLED START*/ //Add additional options here
        alloBallPos = wm->getAlloBallPosition();

        ownPos = wm->getOwnPosition();
        egoBallPos = *alloBallPos->alloToEgo(*ownPos);

        alloTargetPos = CNPoint2D(0, 0);
        egoTargetPos = *alloTargetPos.alloToEgo(*ownPos);

        /*PROTECTED REGION END*/
    }
    DribbleToPoint::~DribbleToPoint()
    {
        /*PROTECTED REGION ID(dcon1414752367688) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DribbleToPoint::run(void* msg)
    {
        /*PROTECTED REGION ID(run1414752367688) ENABLED START*/ //Add additional options here
        msl_simulator::sim_robot_command c;

        double radius = 0.125;
        double w = 4.1;
        double p = atan2(egoTargetPos.y - egoBallPos.y, egoTargetPos.x - egoBallPos.x);

        if (fabs(p) <= 0.115)
        {
            c.velangular = p;
            c.velnormal = -(p * radius);
            c.veltangent = min(egoTargetPos.x * 0.001, 1.0);
            c.spinner = true;
        }
        else
        {
            c.velangular = p * 1.5;
            c.velnormal = -(p * radius) * 1.5;
            c.veltangent = min(egoBallPos.x * 0.05, 1.0);
            c.spinner = true;
        }

//  c.velangular = 4.1;
//  c.velnormal = -(4.1*r);
//  c.veltangent = 0;

        cout << "DribbleToPoint: " << endl;
        this->send(c);

        /*PROTECTED REGION END*/
    }
    void DribbleToPoint::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1414752367688) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1414752367688) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
