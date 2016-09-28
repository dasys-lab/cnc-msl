using namespace std;
#include "Plans/Calibration/RotationCalibrationCalculation.h"

/*PROTECTED REGION ID(inccpp1475074396562) ENABLED START*/ //Add additional includes here
#include <regex>
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
// TODO WHAT THE ACTUAL FUCK
//		string cmd =
//		"gnuplot -e \"f(x) = a*x+b; fit f(x) \\\"sampleRotCalibLog.csv\\\" u 1:2 via a,b; plot \\\"sampleRotCalibLog.csv\\\", f(x), 0; print sprintf(\\\"robotRadius=%f\\\",-b/a)\" 2>&1";
//		std::string gnuplotReturn = ConsoleCommandHelper::exec(cmd.c_str());
//		cout << "length: " << gnuplotReturn.size() << endl;
//		std::regex regex(".*robotRadius=(.*)");
//		std::match_results<std::string::const_iterator> match;
//		cout << "search: " << std::regex_match(gnuplotReturn, match, regex) << endl;
//		cout << "match: " << match[2] << endl;
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
