using namespace std;
#include "Plans/Behaviours/DribbleToPoint.h"

/*PROTECTED REGION ID(inccpp1414752367688) ENABLED START*/ //Add additional includes here
#include <tuple>
#include <MSLWorldModel.h>
using namespace std;
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1414752367688) ENABLED START*/ //initialise static variables here
/*PROTECTED REGION END*/DribbleToPoint::DribbleToPoint() :
    DomainBehaviour("DribbleToPoint")
{
  /*PROTECTED REGION ID(con1414752367688) ENABLED START*/ //Add additional options here
  /*PROTECTED REGION END*/}
DribbleToPoint::~DribbleToPoint()
{
  /*PROTECTED REGION ID(dcon1414752367688) ENABLED START*/ //Add additional options here
  /*PROTECTED REGION END*/}
void DribbleToPoint::run(void* msg)
{
  /*PROTECTED REGION ID(run1414752367688) ENABLED START*/ //Add additional options here
	msl::MSLWorldModel* wm = msl::MSLWorldModel::get();
	  tuple<double, double, double> ownPos = wm->getOwnPosition();
	  pair<double, double> alloBallPos = wm->getBallPosition();
	  pair<double, double> alloTargetPos;
	  alloTargetPos.first = 0;
	  alloTargetPos.second = 0;
	  pair<double, double> egoPos = wm->allo2Ego(alloTargetPos, ownPos);

	  cout << "OwnPosition: ( " << get < 0 > (ownPos) << " ; " << get < 1 > (ownPos) << " ; " << get < 2
	      > (ownPos) << " )\t Ball: ( " << alloBallPos.first << " ; " << alloBallPos.second << " )" << endl;

	  msl_simulator::sim_robot_command c;

	c.velnormal = min(egoPos.second * 0.002, 2.0);
	c.veltangent = min(egoPos.first * 0.002, 2.0);
	c.velangular = 3 * atan2(egoPos.second, egoPos.first);


	  this->send(c);


  /*PROTECTED REGION END*/}
void DribbleToPoint::initialiseParameters()
{
  /*PROTECTED REGION ID(initialiseParameters1414752367688) ENABLED START*/ //Add additional options here
  /*PROTECTED REGION END*/}
/*PROTECTED REGION ID(methods1414752367688) ENABLED START*/ //Add additional methods here


/*PROTECTED REGION END*/} /* namespace alica */
