using namespace std;
#include "Plans/Behaviours/Actuate.h"

/*PROTECTED REGION ID(inccpp1417017518918) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1417017518918) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    Actuate::Actuate() :
            DomainBehaviour("Actuate")
    {
        /*PROTECTED REGION ID(con1417017518918) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    Actuate::~Actuate()
    {
        /*PROTECTED REGION ID(dcon1417017518918) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void Actuate::run(void* msg)
    {
        /*PROTECTED REGION ID(run1417017518918) ENABLED START*/ //Add additional options here
        msl_actuator_msgs::BallHandleCmd bhc;
        msl_actuator_msgs::RawOdometryInfoPtr rodo = wm->getRawOdometryInfo();
        int left, right;

        if (rodo == nullptr)
        {
            cout << "Actuate RODO is empty help" << endl;
            return;
        }
/*
        left = rodo->motion.translation * (1.0 / 40.0) * -1;
        right = rodo->motion.translation * (1.0 / 40.0) * -1;

        bhc.leftMotor = max(min(left, 60), -60);
        bhc.rightMotor = max(min(right, 60), -60);
*/
        bhc.leftMotor = 15;
        bhc.rightMotor = 15;


        this->send(bhc);
        /*PROTECTED REGION END*/
    }
    void Actuate::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1417017518918) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1417017518918) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
