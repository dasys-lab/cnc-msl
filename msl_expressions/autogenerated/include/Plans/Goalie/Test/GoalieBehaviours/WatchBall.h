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
        int currentIndex = 0;
        static const int RING_BUFFER_SIZE = 90;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1447863466691) ENABLED START*/ //Add additional private methods here
        shared_ptr<geometry::CNPosition> me;
        shared_ptr<geometry::CNPoint2D> alloGoalMid;
        shared_ptr<geometry::CNPoint2D> alloFieldCenter;
        shared_ptr<geometry::CNPoint2D> alloBall;
        msl_actuator_msgs::MotionControl mc;
        shared_ptr<geometry::CNPoint2D> oldEgoAlignPoint;
        // ringBuffer for ball impact calculation
        void createCSV(bool movingTowardsGoal);
        int modRingBuffer(int k);
        int ballMovesTowardsGoal(int nrOfPos);
        shared_ptr<geometry::CNPoint2D> calcGoalImpact(shared_ptr<geometry::CNPoint2D> ballPosBuffer[], int nPoints,int puffer);
        shared_ptr<geometry::CNPoint2D> ballPosBuffer[RING_BUFFER_SIZE]; // frequency is 30 Hz, so 2 full iterations
		/*PROTECTED REGION END*/
	};
} /* namespace alica */

#endif /* WatchBall_H_ */
