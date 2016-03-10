#ifndef WatchBall_H_
#define WatchBall_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1447863466691) ENABLED START*/ //Add additional includes here
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"

using namespace msl;
/*PROTECTED REGION END*/
namespace alica
{
	class WatchBall : public DomainBehaviour
	{
	public:
		WatchBall();
		virtual ~WatchBall();
		virtual void run(void* msg);
		/*PROTECTED REGION ID(pub1447863466691) ENABLED START*/ //Add additional public methods here
		/*PROTECTED REGION END*/
	protected:
		virtual void initialiseParameters();
		/*PROTECTED REGION ID(pro1447863466691) ENABLED START*/ //Add additional protected methods here
		static const int SIMULATING = -1; // simulating 1, real life -1
		static const int RING_BUFFER_SIZE = 90;
		static const int nPoints = 10;
		int currentIndex = 0;
		/*PROTECTED REGION END*/
	private:
		/*PROTECTED REGION ID(prv1447863466691) ENABLED START*/ //Add additional private methods here
		msl_actuator_msgs::MotionControl mc;
		int modRingBuffer(int k);
		bool calcGoalImpactY(shared_ptr<geometry::CNPoint2D> &alloTarget, int nPoints, int puffer);
		shared_ptr<geometry::CNPoint2D> oldAlloAlignPoint;
		shared_ptr<geometry::CNPoint2D> oldAlloTarget;
		shared_ptr<geometry::CNPoint2D> ballPosBuffer[RING_BUFFER_SIZE]; // frequency is 30 Hz, so 2 full iterations
		shared_ptr<geometry::CNPoint2D> alloGoalMid;
		shared_ptr<geometry::CNPoint2D> alloFieldCenter;
		/*PROTECTED REGION END*/
	};
} /* namespace alica */

#endif /* WatchBall_H_ */
