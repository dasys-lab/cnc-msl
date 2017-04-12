#ifndef PenaltyAlignAndShoot_H_
#define PenaltyAlignAndShoot_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1431531496053) ENABLED START*/ // Add additional includes here
#include "MSLFootballField.h"

#include <cnc_geometry/Calculator.h>
#include <cnc_geometry/CNPointEgo.h>
#include <cnc_geometry/CNVecAllo.h>
#include <msl_actuator_msgs/BallHandleCmd.h>
#include <msl_actuator_msgs/KickControl.h>
#include <msl_actuator_msgs/MotionControl.h>

#include <sstream>

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
    virtual void run(void *msg);
    /*PROTECTED REGION ID(pub1431531496053) ENABLED START*/ // Add additional public methods here
                                                            /*PROTECTED REGION END*/
  protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1431531496053) ENABLED START*/ // Add additional protected methods here
    double maxVel;
    double angleTolerance;
    double ballDiameter;
    double goalLineLength;
    double robotRadius;
    double wheelSpeed;
    double aimOffset;
    double kickPower;
    double timeForPenaltyShot;
    // 0 = not alignt, 1 = left, 2 = right
    int lastAlignment;
    unsigned long waitBeforeBlindKick;

    // PID variables for alignToPointWithBall
    double defaultRotateP;
    double alignToPointPRot;
    double lastRotError;
    double alignToPointMaxRotation;
    double alignToPointMinRotation;
    //        double lastRotErrorWithBall;
    double alignMaxVel;
    /*PROTECTED REGION END*/
  private:
    /*PROTECTED REGION ID(prv1431531496053) ENABLED START*/ // Add additional private methods here
    msl_actuator_msgs::MotionControl alignToPointWithBall(geometry::CNPointEgo egoAlignPoint,
                                                          geometry::CNPointEgo egoBallPos, double angleTolerance,
                                                          double ballAngleTolerance);
    void readConfigParameters();
        /*PROTECTED REGION END*/};
        } /* namespace alica */

#endif /* PenaltyAlignAndShoot_H_ */
