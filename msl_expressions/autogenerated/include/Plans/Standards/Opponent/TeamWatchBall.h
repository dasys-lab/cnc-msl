#ifndef TeamWatchBall_H_
#define TeamWatchBall_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1457015532224) ENABLED START*/ //Add additional includes here
#include "container/CNPoint2D.h"
/*PROTECTED REGION END*/
namespace alica
{
    class TeamWatchBall : public DomainBehaviour
    {
    public:
        TeamWatchBall();
        virtual ~TeamWatchBall();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1457015532224) ENABLED START*/ //Add additional public methods here
        int maxSend;
        double moveDistance;
        shared_ptr<geometry::CNPoint2D> ballPos;

        int ballMovedOccurrences;
        int maxBallMovedOccurrences;
        int itcounter;
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1457015532224) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1457015532224) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* TeamWatchBall_H_ */
