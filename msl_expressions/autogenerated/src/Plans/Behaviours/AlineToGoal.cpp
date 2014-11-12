using namespace std;
#include "Plans/Behaviours/AlineToGoal.h"

/*PROTECTED REGION ID(inccpp1415205272843) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1415205272843) ENABLED START*/ //initialise static variables here
/*PROTECTED REGION END*/AlineToGoal::AlineToGoal() :
    DomainBehaviour("AlineToGoal")
{
  /*PROTECTED REGION ID(con1415205272843) ENABLED START*/ //Add additional options here
  /*PROTECTED REGION END*/}
AlineToGoal::~AlineToGoal()
{
  /*PROTECTED REGION ID(dcon1415205272843) ENABLED START*/ //Add additional options here
  /*PROTECTED REGION END*/}
void AlineToGoal::run(void* msg)
{
  /*PROTECTED REGION ID(run1415205272843) ENABLED START*/ //Add additional options here
  tuple<double, double, double> ownPos = wm->getOwnPosition();
  pair<double, double> alloEnemyGoal;
  alloEnemyGoal.first = -3000;
  alloEnemyGoal.second = 0;
  pair<double, double> egoEnemyGoal = wm->allo2Ego(alloEnemyGoal, ownPos);

  msl_simulator::sim_robot_command c;

  c.velnormal = 3 * atan2(egoEnemyGoal.second, egoEnemyGoal.first);

  /*PROTECTED REGION END*/}
void AlineToGoal::initialiseParameters()
{
  /*PROTECTED REGION ID(initialiseParameters1415205272843) ENABLED START*/ //Add additional options here
  /*PROTECTED REGION END*/}
/*PROTECTED REGION ID(methods1415205272843) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/} /* namespace alica */
