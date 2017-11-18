using namespace std;
#include "Plans/Calibration/MeasureAndConfigure.h"

/*PROTECTED REGION ID(inccpp1507131462459) ENABLED START*/ //Add additional includes here
#include <math.h>
#include <MSLWorldModel.h>
#include <msl_actuator_msgs/RawOdometryInfo.h>
#include "Plans/Calibration/MeasureDistance.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1507131462459) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    MeasureAndConfigure::MeasureAndConfigure() :
            DomainBehaviour("MeasureAndConfigure")
    {
        /*PROTECTED REGION ID(con1507131462459) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    MeasureAndConfigure::~MeasureAndConfigure()
    {
        /*PROTECTED REGION ID(dcon1507131462459) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void MeasureAndConfigure::run(void* msg)
    {
        /*PROTECTED REGION ID(run1507131462459) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void MeasureAndConfigure::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1507131462459) ENABLED START*/ // Add additional options here
        double laserDistance = MeasureDistance::getLaserDistance(this);
        double odometryDistance = MeasureDistance::getOdometryDistance(this);
        if (max(odometryDistance / laserDistance, laserDistance / odometryDistance) > 1.01)
        {
            // increase wheel radius
            wm->setWheelRadius(wm->getWheelRadius() * (odometryDistance / laserDistance));
            this->setSuccess(false);
        }
        else
        {
            // done
            this->setSuccess(true);
        }
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1507131462459) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
