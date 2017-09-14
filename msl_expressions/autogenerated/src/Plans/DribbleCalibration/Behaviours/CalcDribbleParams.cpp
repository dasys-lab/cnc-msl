using namespace std;
#include "Plans/DribbleCalibration/Behaviours/CalcDribbleParams.h"

/*PROTECTED REGION ID(inccpp1488285993650) ENABLED START*/ //Add additional includes here
#include <boost/lexical_cast.hpp>
#include <Configuration.h>
#include <SystemConfig.h>
#include <string>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1488285993650) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    CalcDribbleParams::CalcDribbleParams() :
            DomainBehaviour("CalcDribbleParams")
    {
        /*PROTECTED REGION ID(con1488285993650) ENABLED START*/ //Add additional options here
        epsilonT = 0;
        velToInput = 0;
        forwardActuatorSpeed = 0;
        backwardActuatorSpeed = 0;
        readConfigParameters();
        /*PROTECTED REGION END*/
    }
    CalcDribbleParams::~CalcDribbleParams()
    {
        /*PROTECTED REGION ID(dcon1488285993650) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void CalcDribbleParams::run(void* msg)
    {
        /*PROTECTED REGION ID(run1488285993650) ENABLED START*/ //Add additional options here
        // TODO: add calculation stuff
//		epsilonT =
//		velToInput =
//		writeConfigParameters();
        this->setSuccess(true);
        return;
        /*PROTECTED REGION END*/
    }
    void CalcDribbleParams::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1488285993650) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1488285993650) ENABLED START*/ //Add additional methods here
    void alica::CalcDribbleParams::readConfigParameters()
    {
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
        forwardActuatorSpeed = (*sc)["DribbleCalibration"]->get<double>(
                "DribbleCalibration.DribbleForward.MeasuredActuatorSpeed", NULL);
        backwardActuatorSpeed = (*sc)["DribbleCalibration"]->get<double>(
                "DribbleCalibration.DribbleBackward.MeasuredActuatorSpeed", NULL);
    }

    void alica::CalcDribbleParams::writeConfigParameters()
    {
        supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
        (*sc)["DribbleCalibration"]->set(boost::lexical_cast < std::string > (epsilonT), "DribbleAlround.epsilonT",
                                         NULL);
        (*sc)["DribbleCalibration"]->set(boost::lexical_cast < std::string > (velToInput), "DribbleAlround.velToInput",
                                         NULL);
    }
/*PROTECTED REGION END*/
} /* namespace alica */
