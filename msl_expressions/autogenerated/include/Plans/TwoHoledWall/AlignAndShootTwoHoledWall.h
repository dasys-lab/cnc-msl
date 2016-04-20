#ifndef AlignAndShootTwoHoledWall_H_
#define AlignAndShootTwoHoledWall_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1417620683982) ENABLED START*/ //Add additional includes here
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

enum HoleMode
{
    toggle = 0, lower = 1, upper = 2
};

/*PROTECTED REGION END*/
namespace alica
{
    class AlignAndShootTwoHoledWall : public DomainBehaviour
    {
    public:
        AlignAndShootTwoHoledWall();
        virtual ~AlignAndShootTwoHoledWall();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1417620683982) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1417620683982) ENABLED START*/ //Add additional protected methods here
        int timesOnTargetCounter = 0;
        HoleMode holeMode;
        bool useLowerHole;
        double maxVel;
        double pRot;
        double dRot;
        double lastRotError;
        double minRot;
        double maxRot;
        double angleTolerance, ballAngleTolerance;
        bool disableKicking;
        bool kicked;
        int iterationsAfterKick;

        geometry::CNPoint3D higherHole;
        geometry::CNPoint3D lowerHole;

        int timesOnTargetThreshold;
        int wheelSpeed;
        double voltage4shoot;

        vector<shared_ptr<geometry::CNPoint2D>> highKickList;
        vector<shared_ptr<geometry::CNPoint2D>> lowKickList;

        unsigned short setKickPower(double distance);

        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1417620683982) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* AlignAndShootTwoHoledWall_H_ */
