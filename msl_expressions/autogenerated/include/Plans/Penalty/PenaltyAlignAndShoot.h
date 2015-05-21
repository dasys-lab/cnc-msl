#ifndef PenaltyAlignAndShoot_H_
#define PenaltyAlignAndShoot_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1431531496053) ENABLED START*/ //Add additional includes here
#include "container/CNPoint2D.h"
#include "container/CNPoint3D.h"
#include "MSLFootballField.h"
#include <sstream>
#include <container/CNVelocity2D.h>
#include <GeometryCalculator.h>
#include "msl_actuator_msgs/MotionControl.h"
#include "msl_actuator_msgs/KickControl.h"
#include "msl_actuator_msgs/BallHandleCmd.h"

using namespace msl;
using namespace msl_actuator_msgs;

/*PROTECTED REGION END*/
namespace alica
{
    class PenaltyAlignAndShoot : public DomainBehaviour
    {
    public:
        PenaltyAlignAndShoot();
        virtual ~PenaltyAlignAndShoot();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1431531496053) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1431531496053) ENABLED START*/ //Add additional protected methods here
        msl::MSLFootballField* field;
        double maxVel;
        double pRot;
        double dRot;
        double lastRotError;
        double minRot;
        double maxRot;
        double angleTolerance, ballAngleTolerance;
        double ballDiameter;
        double goalLineLength;
        double robotDiameter;
        double wheelSpeed;
        double aimOffset;
        double kickPower;
        //0 = not alignt, 1 = left, 2 = right
        int lastAlignment;
        int startTime;

        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1431531496053) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* PenaltyAlignAndShoot_H_ */
