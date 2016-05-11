#ifndef DribbleControl_H_
#define DribbleControl_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1449742071382) ENABLED START*/ //Add additional includes here
#include "CubicSplineInterpolation/Spline.h"
/*PROTECTED REGION END*/
namespace alica
{
    class DribbleControl : public DomainBehaviour
    {
    public:
        DribbleControl();
        virtual ~DribbleControl();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1449742071382) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1449742071382) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1449742071382) ENABLED START*/ //Add additional private methods here
        double handlerSpeedSummand = 0;
        void readConfigParameters();
        bool pullNoMatterWhat = false;
        bool controlNoMatterWhat = false;
        bool haveBall = false;
        bool hadBefore = false;
        int itcounter = 0;
        double handlerSpeedFactor = 0.0;
        double speedNoBall = 0.0;
        double slowTranslation = 0.0;
        double slowTranslationWheelSpeed = 0.0;
        double curveRotationFactor = 0.0;
        double orthoDriveFactor = 0;
        splines::spline forwardSpeedSpline;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DribbleControl_H_ */
