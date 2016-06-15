#ifndef KickOffPassDefault_H_
#define KickOffPassDefault_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1438778042140) ENABLED START*/ //Add additional includes here
#include "container/CNPoint2D.h"
#include <GeometryCalculator.h>
/*PROTECTED REGION END*/
namespace alica
{
	class KickOffPassDefault : public DomainBehaviour
	{
	public:
		KickOffPassDefault();
		virtual ~KickOffPassDefault();
		virtual void run(void* msg);
		/*PROTECTED REGION ID(pub1438778042140) ENABLED START*/ //Add additional public methods here
		/*PROTECTED REGION END*/
	protected:
		virtual void initialiseParameters();
		/*PROTECTED REGION ID(pro1438778042140) ENABLED START*/ //Add additional protected methods here
		string taskName;
		/*PROTECTED REGION END*/
	private:
		/*PROTECTED REGION ID(prv1438778042140) ENABLED START*/ //Add additional private methods here
		double timeForPass;
		double waitBeforeBlindKick;

		// PID variables for alignToPointWithBall
		double defaultRotateP;
		double alignToPointpRot;
		double lastRotError;
		double alignToPointMaxRotation;
		double alignToPointMinRotation;
		double lastRotErrorWithBall;
		double alignMaxVel;

		msl_actuator_msgs::MotionControl alignToPointWithBall(shared_ptr<geometry::CNPoint2D> egoAlignPoint,
																shared_ptr<geometry::CNPoint2D> egoBallPos,
																double angleTolerance, double ballAngleTolerance);
		void readConfigParameters();
		/*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* KickOffPassDefault_H_ */
