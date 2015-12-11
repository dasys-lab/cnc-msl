#ifndef CalcCalib_H_
#define CalcCalib_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1446033324019) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
    class CalcCalib : public DomainBehaviour
    {
    public:
        CalcCalib();
        virtual ~CalcCalib();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1446033324019) ENABLED START*/ //Add additional public methods here
        double calibPosMotionY;
        double calibPosMotionX;
        double calibOldPosMotionX = 0;
        double calibOldPosMotionY = 0;
        double wayX;
        double wayY;
        double correctedWayX;
        double correctedWayY;
        double diffX;
        double diffY;

        shared_ptr<geometry::CNPosition> calibPosMotion;
        shared_ptr<geometry::CNPosition> calibOldPosMotion;
        shared_ptr<geometry::CNPosition> calibPosVision;
        shared_ptr<geometry::CNPosition> calibOldPosVision;
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1446033324019) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1446033324019) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* CalcCalib_H_ */
