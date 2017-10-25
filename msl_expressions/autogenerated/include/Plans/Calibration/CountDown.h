#ifndef CountDown_H_
#define CountDown_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1508940993195) ENABLED START*/ //Add additional includes here
#include <chrono>
#include <ratio>
/*PROTECTED REGION END*/
namespace alica
{
    class CountDown : public DomainBehaviour
    {
    public:
        CountDown();
        virtual ~CountDown();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1508940993195) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1508940993195) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1508940993195) ENABLED START*/ //Add additional private methods here
        std::chrono::system_clock::time_point begin_time;
        shared_ptr<std::chrono::milliseconds> countDownTime;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* CountDown_H_ */
