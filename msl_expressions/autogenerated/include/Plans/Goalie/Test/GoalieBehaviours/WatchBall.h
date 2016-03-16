#ifndef WatchBall_H_
#define WatchBall_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1447863466691) ENABLED START*/ //Add additional includes here
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"
#include <gsl/gsl_math.h>
#include <string>

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
        static const int SIMULATING = 1; // simulating 1, real life -1
        static const int BALL_BUFFER_SIZE = 10;
        static const int TARGET_BUFFER_SIZE = 3;
        static const int GOALIE_SIZE = 900;
        static const string LEFT;
        static const string MID;
        static const string RIGHT;
        shared_ptr<geometry::CNPoint2D> alloFieldCntr = MSLFootballField::posCenterMarker();
        shared_ptr<geometry::CNPoint2D> alloAlignPt = alloFieldCntr;
        bool writeLog = false;
        int ballIndex = 0;
        int targetIndex = 0;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1447863466691) ENABLED START*/ //Add additional private methods here
        msl_actuator_msgs::MotionControl mc;
        shared_ptr<geometry::CNPosition> me;
        void moveInsideGoal(shared_ptr<geometry::CNPoint2D> alloBall, shared_ptr<geometry::CNPosition> me);
        string fitTargetY(double targetY);
        void sendMC(string targetPos);
        int modRingBuffer(int k, int bufferSize);
        shared_ptr<geometry::CNPoint2D> calcGoalImpactY(int nPoints);
        shared_ptr<geometry::CNPoint2D> alloGoalLeft;
        shared_ptr<geometry::CNPoint2D> alloGoalRight;
        shared_ptr<geometry::CNPoint2D> alloGoalMid;
        shared_ptr<geometry::CNPoint2D> ballPosBuffer[BALL_BUFFER_SIZE];
        shared_ptr<geometry::CNPoint2D> targetPosBuffer[TARGET_BUFFER_SIZE];
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* WatchBall_H_ */
