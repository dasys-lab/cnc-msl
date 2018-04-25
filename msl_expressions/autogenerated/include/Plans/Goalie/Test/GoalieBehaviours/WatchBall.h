#ifndef WatchBall_H_
#define WatchBall_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1447863466691) ENABLED START*/ // Add additional includes here
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"
#include <MSLFootballField.h>
#include <MSLWorldModel.h>
#include <RingBuffer.h>
#include <msl_actuator_msgs/MotionControl.h>
#include <msl_robot/robotmovement/MovementQuery.h>
#include <string>

using msl_actuator_msgs::MotionControl;
using namespace msl;
/*PROTECTED REGION END*/
namespace alica
{
class WatchBall : public DomainBehaviour
{
  public:
    WatchBall();
    virtual ~WatchBall();
    virtual void run(void *msg);
    /*PROTECTED REGION ID(pub1447863466691) ENABLED START*/ // Add additional public methods here
    double fitTargetY(double targetY);
    /*PROTECTED REGION END*/
  protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1447863466691) ENABLED START*/ // Add additional protected methods here

	// Values used by methods and behaviour
	shared_ptr<geometry::CNPosition> ownPos;
	shared_ptr<geometry::CNPoint2D> alloGoalLeft;
    shared_ptr<geometry::CNPoint2D> alloGoalRight;
    shared_ptr<geometry::CNPoint2D> alloGoalMid;
    shared_ptr<geometry::CNPoint2D> prevTarget;
    shared_ptr<geometry::CNPoint2D> alloBall;

	// Config parameters
    int maxVariance;
    int goalieSize;
    int snapDistance;
    double maxRotationRad;
    double goalWidth;

	// ballPositions is a RingBuffer used to predict the impact point.
    msl::RingBuffer<geometry::CNPoint2D> *ballPositions;

	// calcGoalImpactY uses the RingBuffer to calculate the Y impact of
	// the ball at the goal line.
    double calcGoalImpactY();

	// updateGoalPosition uses the laser scanner to update the goals mid
	// position.
    void updateGoalPosition();

	// query is the MovementQuery for the robot to drive to the goal.
    shared_ptr<msl::MovementQuery> query;

	// Helper functions
	
	// clampRotations clamps or limits the rotation to maxRot.
	// Given the robots rotation ownTheta and the demanded mcRotation, don't rotate
	// if the the robot would exceed the max rotation given maxRot.
	double clampRotation(double mcRotation, double ownTheta, double maxRot);

	// driveTo drives to the given target but respects the goal area
	MotionControl driveAndAlignTo(
			shared_ptr<geometry::CNPoint2D> target,
			shared_ptr<geometry::CNPoint2D> alloAlginPoint
			);

	// Make foxy move faster, but not too fast xD
	MotionControl faster(const MotionControl in);

	// Return the mirrored ownPos for alignment.
	shared_ptr<geometry::CNPoint2D> mirroredOwnPos();

    /*PROTECTED REGION END*/
  private:
    /*PROTECTED REGION ID(prv1447863466691) ENABLED START*/ // Add additional private methods here
        /*PROTECTED REGION END*/};
        } /* namespace alica */

#endif /* WatchBall_H_ */
