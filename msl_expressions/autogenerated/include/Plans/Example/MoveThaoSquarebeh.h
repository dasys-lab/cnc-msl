#ifndef MoveThaoSquarebeh_H_
#define MoveThaoSquarebeh_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1450269578569) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class MoveThaoSquarebeh : public DomainBehaviour
    {
    public:
        MoveThaoSquarebeh();
        virtual ~MoveThaoSquarebeh();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1450269578569) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1450269578569) ENABLED START*/ //Add additional protected methods here
        int count;
        shared_ptr<geometry::CNPosition> ownPos;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1450269578569) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* MoveThaoSquarebeh_H_ */
