using namespace std;
#ifndef Actuate_H_
#define Actuate_H_

#include "DomainBehaviour.h"

/*PROTECTED REGION ID(inc1417017518918) ENABLED START*/ //Add additional includes here
#include <iostream>
#include <list>
/*PROTECTED REGION END*/
namespace alica
{
    class Actuate : public DomainBehaviour
    {
    public:
        Actuate();
        virtual ~Actuate();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1417017518918) ENABLED START*/ //Add additional public methods here

        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1417017518918) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1417017518918) ENABLED START*/ //Add additional private methods here

        list<double> arithmeticAverageBox;

        /*PROTECTED REGION END*/
    };
} /* namespace alica */

#endif /* Actuate_H_ */
