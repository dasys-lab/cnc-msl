using namespace std;
#include "Plans/Calibration/DriveForward.h"

/*PROTECTED REGION ID(inccpp1507131193237) ENABLED START*/ //Add additional includes here
#include <MSLWorldModel.h>
#include <LaserScannerPosition.h>
#include <msl_actuator_msgs/RawOdometryInfo.h>
#include <engine/RunningPlan.h>
#include "msl_actuator_msgs/MotionControl.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1507131193237) ENABLED START*/ //initialise static variables here
	geometry::CNPoint2D DriveForward::startPositionLaser;
	geometry::CNPoint2D DriveForward::startPositionMotion;
    /*PROTECTED REGION END*/
    DriveForward::DriveForward() :
            DomainBehaviour("DriveForward")
    {
        /*PROTECTED REGION ID(con1507131193237) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    DriveForward::~DriveForward()
    {
        /*PROTECTED REGION ID(dcon1507131193237) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void DriveForward::run(void* msg)
    {
        /*PROTECTED REGION ID(run1507131193237) ENABLED START*/ //Add additional options here
    	if (this->isSuccess())
		{
			return;
		}
    	msl_actuator_msgs::MotionControl mc;
    	mc.motion.rotation = 0.0;
    	mc.motion.angle = 0.0;
    	mc.motion.translation = 1.0;this->runningPlan->
    	send(mc);

    	if (std::chrono::system_clock::now() - begin_time < FIVE_SECONDS)
    	{
    		this->setSuccess(true);
    	}
        /*PROTECTED REGION END*/
    }
    void DriveForward::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1507131193237) ENABLED START*/ //Add additional options here
    	DriveForward::startPositionMotion.x = wm->rawOdometry->position.x;
    	DriveForward::startPositionMotion.y = wm->rawOdometry->position.y;
    	shared_ptr<geometry::CNPoint2D> laserPosition = wm->laserScannerPosition->getPosition();
    	DriveForward::startPositionLaser.x = laserPosition->x;
    	DriveForward::startPositionLaser.y = laserPosition->y;
    	begin_time = std::chrono::system_clock::now();

    	//startPositionLaser.x = wm->laserScannerPosition->getPosition()->x;
    	//startPositionLaser.y = wm->laserScannerPosition->getPosition()->y;
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1507131193237) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
