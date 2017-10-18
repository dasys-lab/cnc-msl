#ifndef DriveForward_H_
#define DriveForward_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1507131193237) ENABLED START*/ //Add additional includes here
#include <chrono>
#include <ratio>
#include <container/CNPoint2D.h>
/*PROTECTED REGION END*/
namespace alica
{
    class DriveForward : public DomainBehaviour
    {
    public:
        DriveForward();
        virtual ~DriveForward();
        static geometry::CNPoint2D startPositionMotion;
        static geometry::CNPoint2D startPositionLaser;
        virtual void run(void* msg);
        const std::chrono::duration<int,std::ratio<5> > FIVE_SECONDS (1);
        /*PROTECTED REGION ID(pub1507131193237) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1507131193237) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1507131193237) ENABLED START*/ //Add additional private methods here
    	std::chrono::system_clock::time_point begin_time;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DriveForward_H_ */
