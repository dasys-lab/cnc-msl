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
    	if (!this->isSuccess())
    	{
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

			/* I like C! */
			// match plotted value
			const int max_line_size = 400;
			const int output_size = gnuplotReturn.size() + 1;
			const char *ret = gnuplotReturn.c_str();
			char output[output_size];
			char *line;
			char *lastline;
			char *radiusStr;

			strncpy(output, gnuplotReturn.c_str(), output_size);

			// Get last line
			line = strtok(output, "\n");
			while ((line = strtok(NULL, "\n")) != NULL)
				lastline = line;

			// Get the radius as string
			strtok(lastline, "="); // eats robotRadius
			radiusStr = strtok(NULL, "=");
			if (radiusStr == NULL) {
				cerr << "ERROR parsing gnuplot output" << endl;
				this->setSuccess(true); // not really tho
				return;
			}

			// Convert it to double
			double calculatedValue = strtod(radiusStr, NULL);
			if (calculatedValue <= 0 || calculatedValue > 500) {
				// Something went wrong during parsing when value = 0
				cerr << "ERROR parsing robot radius OR robot radius not appropriate" << endl;
				cerr << "Calculated Robot Radius: " << calculatedValue << endl;
				this->setSuccess(true); // not really tho
				return;
			}

			cout << "SUCCESS!" << endl;
			cout << "Calculated Robot Radius: " << calculatedValue << endl;

			wm->setRobotRadius(calculatedValue);
			cout << "OH SHIT IT WORKED!" << endl;

			this->setSuccess(true);
    	}
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
