using namespace std;
#include "Plans/Calibration/RotationCalibrationCalculation.h"

/*PROTECTED REGION ID(inccpp1475074396562) ENABLED START*/ //Add additional includes here
#include <regex>
#include <sstream>
#include <MSLWorldModel.h>
#include "ConsoleCommandHelper.h"
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
		string cmd =
				"gnuplot -persist -e \"f(x) = a*x+b; fit f(x) \\\"sampleRotCalibLog.csv\\\" u 1:2 via a,b; plot \\\"sampleRotCalibLog.csv\\\", f(x), 0; print sprintf(\\\"robotRadius=%f\\\",-b/a)\" 2>&1";
		string gnuplotReturn = supplementary::ConsoleCommandHelper::exec(cmd.c_str());
		// match plotted value
		regex regex(".*robotRadius=(.*)");
		match_results < string::const_iterator > match;
		string matchedValue = match[1];
		double calculatedValue;
		// here we omit the unnecessary '=' that is captured by the regex
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
