using namespace std;
#include "Plans/Behaviours/DriveForward.h"

/*PROTECTED REGION ID(inccpp1414427325853) ENABLED START*/ //Add additional includes here
#include <tuple>

using namespace std;

	pair<double, double> alica::DriveForward::allo2Ego(pair<double, double>& p, tuple<double, double, double>& ownPos)
	{
		pair<double, double> ego;

		double x = p.first - get<0>(ownPos);
		double y = p.second - get<1>(ownPos);

		double angle = atan2(y, x) - get<2>(ownPos);
		double dist = sqrt(x*x + y*y);

		ego.first = cos(angle)*dist;
		ego.second = sin(angle)*dist;

		return ego;
	}

/*PROTECTED REGION END*/
namespace alica
{
	DriveForward::DriveForward() :
			msl::MSLBehaviour("DriveForward")
	{
		/*PROTECTED REGION ID(con1414427325853) ENABLED START*/ //Add additional options here

	/*PROTECTED REGION END*/}
DriveForward::~DriveForward()
{
	/*PROTECTED REGION ID(dcon1414427325853) ENABLED START*/ //Add additional options here
	/*PROTECTED REGION END*/}
void DriveForward::run(void* msg)
{
	/*PROTECTED REGION ID(run1414427325853) ENABLED START*/ //Add additional options here
	msl::MSLWorldModel* wm = msl::MSLWorldModel::get();
	tuple<double, double, double> ownPos = wm->getOwnPosition();
	pair<double, double> alloBallPos = wm->getBallPosition();
	pair<double, double> egoBallPos = this->allo2Ego(alloBallPos, ownPos);

	cout << "OwnPosition: ( " << get<0>(ownPos) << " ; " << get<1>(ownPos) << " ; " << get<2>(ownPos) << " )\t Ball: ( "
			<< alloBallPos.first << " ; " << alloBallPos.second << " )" << endl;

	msl_simulator::sim_robot_command c;
	c.velnormal = 0;
	c.veltangent = min(egoBallPos.first*0.01, 1.0);
	c.velangular = 10*atan2(egoBallPos.second, egoBallPos.first);

	this->send(c);

	/*PROTECTED REGION END*/}
void DriveForward::initialiseParameters()
{
	/*PROTECTED REGION ID(initialiseParameters1414427325853) ENABLED START*/ //Add additional options here
	/*PROTECTED REGION END*/}
} /* namespace alica */
