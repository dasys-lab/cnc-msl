using namespace std;
#include "Plans/DribbleCalibration/Behaviours/CalcDribbleParams.h"

/*PROTECTED REGION ID(inccpp1489492250448) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1489492250448) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	CalcDribbleParams::CalcDribbleParams() :
			DomainBehaviour("CalcDribbleParams")
	{
		/*PROTECTED REGION ID(con1489492250448) ENABLED START*/ //Add additional options here
		measuredForwardSpeed = 0;
		measuredBackwardSpeed = 0;
		velToInput = 0;
		epsilonT = 0;
		calibTrans = 0;
		/*PROTECTED REGION END*/
	}
	CalcDribbleParams::~CalcDribbleParams()
	{
		/*PROTECTED REGION ID(dcon1489492250448) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void CalcDribbleParams::run(void* msg)
	{
		/*PROTECTED REGION ID(run1489492250448) ENABLED START*/ //Add additional options here
		epsilonT = (measuredBackwardSpeed + measuredForwardSpeed)/(measuredBackwardSpeed - measuredForwardSpeed);
		velToInput = (measuredForwardSpeed)/(calibTrans + (1 - epsilonT));
		/*PROTECTED REGION END*/
	}
	void CalcDribbleParams::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1489492250448) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1489492250448) ENABLED START*/ //Add additional methods here
	void CalcDribbleParams::readConfigParams()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		measuredForwardSpeed = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleForward.MeasuredActuatorSpeed", NULL);
		measuredBackwardSpeed = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.DribbleBackward.MeasuredActuatorSpeed", NULL);
		calibTrans = (*sc)["DribbleCalibration"]->get<double>("DribbleCalibration.Default.EndTranslation", NULL);
	}

	void CalcDribbleParams::writeConfigParams()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		(*sc)["DribbleAlround"]->set(boost::lexical_cast<std::string>(velToInput), "DribbleAlround.velToInput", NULL);
		(*sc)["DribbleAlround"]->set(boost::lexical_cast<std::string>(epsilonT), "DribbleAlround.epsilonT", NULL);
	}
/*PROTECTED REGION END*/
} /* namespace alica */
