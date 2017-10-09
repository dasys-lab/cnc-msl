#ifndef DribbleEmergencyKick_H_
#define DribbleEmergencyKick_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1457706800035) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class DribbleEmergencyKick : public DomainBehaviour
    {
    public:
        DribbleEmergencyKick();
        virtual ~DribbleEmergencyKick();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1457706800035) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1457706800035) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1457706800035) ENABLED START*/ //Add additional private methods here
        geometry::CNPointAllo ballPos;
        int kickpower;
        bool safeKick;
        bool haveKicked;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DribbleEmergencyKick_H_ */
