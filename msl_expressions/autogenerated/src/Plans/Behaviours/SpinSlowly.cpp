using namespace std;
#include "Plans/Behaviours/SpinSlowly.h"

/*PROTECTED REGION ID(inccpp1435159253296) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1435159253296) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	SpinSlowly::SpinSlowly() :
			DomainBehaviour("SpinSlowly")
	{
		/*PROTECTED REGION ID(con1435159253296) ENABLED START*/ //Add additional options here
		center = make_shared<geometry::CNPoint2D>(0, 0);
		/*PROTECTED REGION END*/
	}
	SpinSlowly::~SpinSlowly()
	{
		/*PROTECTED REGION ID(dcon1435159253296) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void SpinSlowly::run(void* msg)
	{
		/*PROTECTED REGION ID(run1435159253296) ENABLED START*/ //Add additional options here
		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData.getOwnPositionVision();
		if (ownPos == nullptr)
		{
			return;
		}
		alpha = center->alloToEgo(*ownPos)->angleTo();
		msl_actuator_msgs::MotionControl mc;
		//        mc.motion.angle = M_PI / 4;

		if (abs(startAngle - alpha) < epsilon && counter >10)
		{
			mc.motion.rotation = 0;
		}
		else
		{
			counter++;
			mc.motion.rotation = M_PI / 4;
		}
		if (startAngle == 999)
		{
			startAngle = alpha;
		}
		send(mc);

		/*PROTECTED REGION END*/
	}
	void SpinSlowly::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1435159253296) ENABLED START*/ //Add additional options here
		alpha = 0;
		startAngle = 999;
		counter = 0;
		/*PROTECTED REGION END*/
	}
/*PROTECTED REGION ID(methods1435159253296) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
