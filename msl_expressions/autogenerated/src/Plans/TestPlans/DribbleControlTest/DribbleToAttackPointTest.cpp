using namespace std;
#include "Plans/TestPlans/DribbleControlTest/DribbleToAttackPointTest.h"

/*PROTECTED REGION ID(inccpp1498664309837) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1498664309837) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	DribbleToAttackPointTest::DribbleToAttackPointTest() :
			DomainBehaviour("DribbleToAttackPointTest")
	{
		/*PROTECTED REGION ID(con1498664309837) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	DribbleToAttackPointTest::~DribbleToAttackPointTest()
	{
		/*PROTECTED REGION ID(dcon1498664309837) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void DribbleToAttackPointTest::run(void* msg)
	{
		/*PROTECTED REGION ID(run1498664309837) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void DribbleToAttackPointTest::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1498664309837) ENABLED START*/ //Add additional options here
		wheelSpeed = -80;
		lastClosesOpp = nullptr;
		lastRotError = 0;
		for (int i = 0; i < pastRotation.size(); i++)
		{
			pastRotation.at(i) = 0;
		}
		counter = -1;
		/*PROTECTED REGION END*/
	}
/*PROTECTED REGION ID(methods1498664309837) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
