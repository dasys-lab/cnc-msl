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
        double calibPosMotionY = 0;
        double calibPosMotionX = 0;
        double calibOldPosMotionX = 0;
        double calibOldPosMotionY = 0;
        double calibCoefficientX;
        double calibCoefficientY;
        double length = 0;
        double correctedWayX = 0;
        double correctedWayY = 0;
        double correctedPosX = 0;
        double correctedPosY = 0;
        double oldCorrectedPosX = 0;
        double oldCorrectedPosY = 0;
        double diffX;
        double diffY;
        double lengthSegment = 0;
        CalibrationCoefficient calibCoeff;
        int calibCounter = 0;

        //------------------------------

        double errorTestMotionPosX = 0;
        double errorTestMotionPosY = 0;
		double errorTestVisionPosX = 0;
		double errorTestVisionPosY = 0;
		double oldErrorTestMotionPosX = 0;
		double oldErrorTestMotionPosY = 0;
		double oldErrorTestVisionPosX = 0;
		double oldErrorTestVisionPosY = 0;
		int errorCounter = 0;
		int timeCounter = 0;

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
