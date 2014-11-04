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
  pair<double, double> alloBallPos = wm->getBallPosition();

  tuple<double, double, double> ownPos = wm->getOwnPosition();
  pair<double, double> egoBallPos = wm->allo2Ego(alloBallPos, ownPos);

  pair<double, double> alloTargetPos;
  alloTargetPos.first = 0;
  alloTargetPos.second = 0;
  pair<double, double> egoPos = wm->allo2Ego(alloTargetPos, ownPos);



  msl_simulator::sim_robot_command c;

  double r = 0.125;
  double w = 4.1;
  double p = atan2(egoPos.second-egoBallPos.second, egoPos.first-egoBallPos.first);


	  if(fabs(p)<=0.115) {
		  c.velangular = p;
		  c.velnormal = -(p*r);
		  c.veltangent = min(egoPos.first * 0.001, 1.0);
		  c.spinner = true;
	  } else {
		   c.velangular = p*1.5;
		   c.velnormal = -(p*r)*1.5;
		   c.veltangent = min(egoBallPos.first * 0.05, 1.0);
		   c.spinner = true;
	  }


//  c.velangular = 4.1;
//  c.velnormal = -(4.1*r);
//  c.veltangent = 0;

	  cout << "DribbleToPoint: " << "OwnPosition: ( " << get < 0 > (ownPos) << " ; " << get < 1 > (ownPos) << " ; " << get
	        < 2 > (ownPos) << " )\t Ball: ( " << alloBallPos.first << " ; " << alloBallPos.second << " )" << "WINKEL: " << p << endl;
  this->send(c);

  /*PROTECTED REGION END*/}
void DribbleToPoint::initialiseParameters()
{
  /*PROTECTED REGION ID(initialiseParameters1414752367688) ENABLED START*/ //Add additional options here
  /*PROTECTED REGION END*/}
/*PROTECTED REGION ID(methods1414752367688) ENABLED START*/ //Add additional methods here
bool haveBalll()
{

  return false;
}

void getBall()
{

}
/*PROTECTED REGION END*/} /* namespace alica */
