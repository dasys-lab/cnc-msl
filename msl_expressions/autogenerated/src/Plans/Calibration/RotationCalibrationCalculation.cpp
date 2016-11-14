using namespace std;
#include "Plans/Calibration/RotationCalibrationCalculation.h"

/*PROTECTED REGION ID(inccpp1475074396562) ENABLED START*/ //Add additional includes here
#include <regex>
#include <sstream>
#include <MSLWorldModel.h>
#include "ConsoleCommandHelper.h"
#include <SystemConfig.h>
#include <FileSystem.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1475074396562) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    RotationCalibrationCalculation::RotationCalibrationCalculation() :
            DomainBehaviour("RotationCalibrationCalculation")
    {
        /*PROTECTED REGION ID(con1475074396562) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    RotationCalibrationCalculation::~RotationCalibrationCalculation()
    {
        /*PROTECTED REGION ID(dcon1475074396562) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void RotationCalibrationCalculation::run(void* msg)
    {
        /*PROTECTED REGION ID(run1475074396562) ENABLED START*/ //Add additional options here
        // call gnuplot for maximum convenience!!
        stringstream cmd;
        string logfile = supplementary::FileSystem::combinePaths(sc->getLogPath(), "RotationCalibration.log");
        cmd << "gnuplot -persist -e \"f(x) = a*x+b; fit f(x) \\\"";
        cmd << logfile;
        cmd << "\\\" u 1:2 via a,b; plot \\\"";
        cmd << logfile;
        cmd << "\\\", f(x), 0; print sprintf(\\\"robotRadius=%f\\\",-b/a)\" 2>&1";

        cout << cmd.str() << endl;
        string gnuplotReturn = supplementary::ConsoleCommandHelper::exec(cmd.str().c_str());
        // match plotted value
        regex regex(".*robotRadius=(.*)");
        match_results < string::const_iterator > match;
        regex_match(gnuplotReturn, match, regex);
        string matchedValue = match[1];
        cout << match[0] << endl;
        double calculatedValue;
        // here we omit the unnecessary '=' that is captured by the regex
        // cout << "MATCHED VALUE=" << matchedValue << endl << "MATCH1=" << match[1] << endl << "MATCH2=" << match[2] << endl;
        if (matchedValue.size() > 0)
        {
            if (matchedValue.at(0) == '=')
            {
                matchedValue = matchedValue.substr(1);
            }
            stringstream str(matchedValue);
            str >> calculatedValue;
            if (calculatedValue > 0 && calculatedValue < 1000)
            {
                wm->setRobotRadius(calculatedValue);
            }
            else
            {
                cout << "OH SHIT CALCULATED VALUE IS " << calculatedValue << "!!! <-- ZHAT SEEMS SUSPICOIUS" << endl;
            }
        }
        else
        {
            cout << "OH SHIT GNUPLOT RETURN MATCHING FAILED (did you run ssh -X?)" << endl;
        }
        this->setSuccess(true);

        /*PROTECTED REGION END*/
    }
    void RotationCalibrationCalculation::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1475074396562) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1475074396562) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
