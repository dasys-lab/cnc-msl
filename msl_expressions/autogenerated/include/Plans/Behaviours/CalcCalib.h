#ifndef CalcCalib_H_
#define CalcCalib_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1446033324019) ENABLED START*/ //Add additional includes here
#define calibSign(a,b) (a > b ? 1: -1)
#include "msl_actuator_msgs/CalibrationCoefficient.h"
using namespace msl_actuator_msgs;
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
        double calibOldPosMotionX;
        double calibOldPosMotionY;
        double calibCoefficientX;
        double calibCoefficientY;
        double oldCalibCoefficientX;
        double oldCalibCoefficientY;
        double correctedWayX;
        double correctedWayY;
        double correctedPosX;
        double correctedPosY;
        double diffX;
        double diffY;
        double lengthSegment;
        int calibCounter;
        CalibrationCoefficient calibCoeff;

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
