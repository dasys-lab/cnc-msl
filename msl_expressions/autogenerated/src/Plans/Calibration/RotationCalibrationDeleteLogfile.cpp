using namespace std;
#include "Plans/Calibration/RotationCalibrationDeleteLogfile.h"

/*PROTECTED REGION ID(inccpp1479315274633) ENABLED START*/ //Add additional includes here
#include <cstdio>
#include <SystemConfig.h>
#include <FileSystem.h>
#include <MSLWorldModel.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1479315274633) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    RotationCalibrationDeleteLogfile::RotationCalibrationDeleteLogfile() :
            DomainBehaviour("RotationCalibrationDeleteLogfile")
    {
        /*PROTECTED REGION ID(con1479315274633) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    RotationCalibrationDeleteLogfile::~RotationCalibrationDeleteLogfile()
    {
        /*PROTECTED REGION ID(dcon1479315274633) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void RotationCalibrationDeleteLogfile::run(void* msg)
    {
        /*PROTECTED REGION ID(run1479315274633) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void RotationCalibrationDeleteLogfile::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1479315274633) ENABLED START*/ //Add additional options here
        std::string logfilePath = supplementary::FileSystem::combinePaths(sc->getLogPath(), "RotationCalibration.log");
        std::remove(logfilePath.c_str());
//        RotateOnce::measurements[1] = new geometry::CNPoint2D(wm->getRobotRadius(), 0);
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1479315274633) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
