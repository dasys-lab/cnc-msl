using namespace std;
#include "Plans/Behaviours/GetBall.h"

/*PROTECTED REGION ID(inccpp1414828300860) ENABLED START*/ //Add additional includes here
#include <tuple>
#include <MSLWorldModel.h>
using namespace std;
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1414828300860) ENABLED START*/ //initialise static variables here
/*PROTECTED REGION END*/GetBall::GetBall() :
    DomainBehaviour("GetBall")
{
  /*PROTECTED REGION ID(con1414828300860) ENABLED START*/ //Add additional options here
  /*PROTECTED REGION END*/}
GetBall::~GetBall()
{
  /*PROTECTED REGION ID(dcon1414828300860) ENABLED START*/ //Add additional options here
  /*PROTECTED REGION END*/}
void GetBall::run(void* msg)
{
  /*PROTECTED REGION ID(run1414828300860) ENABLED START*/ //Add additional options here
  tuple<double, double, double> ownPos = wm->getOwnPosition();
  pair<double, double> alloBallPos = wm->getBallPosition();
  pair<double, double> egoBallPos = wm->allo2Ego(alloBallPos, ownPos);

  cout << "DriveForward: " << "OwnPosition: ( " << get < 0 > (ownPos) << " ; " << get < 1 > (ownPos) << " ; " << get < 2
      > (ownPos) << " )\t Ball: ( " << alloBallPos.first << " ; " << alloBallPos.second << " )" << endl;

  msl_simulator::sim_robot_command c;

  c.velnormal = min(egoBallPos.second * 0.002, 2.0);
  c.veltangent = min(egoBallPos.first * 0.002, 2.0);
  c.velangular = 3 * atan2(egoBallPos.second, egoBallPos.first);

  this->send(c);
  /*PROTECTED REGION END*/}
void GetBall::initialiseParameters()
{
  /*PROTECTED REGION ID(initialiseParameters1414828300860) ENABLED START*/ //Add additional options here
  /*PROTECTED REGION END*/}
/*PROTECTED REGION ID(methods1414828300860) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/} /* namespace alica */
