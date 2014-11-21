using namespace std;
#include "Plans/Behaviours/Stop.h"

/*PROTECTED REGION ID(inccpp1413992604875) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
/*PROTECTED REGION ID(staticVars1413992604875) ENABLED START*/ //initialise static variables here
/*PROTECTED REGION END*/Stop::Stop() :
    DomainBehaviour("Stop")
{
  /*PROTECTED REGION ID(con1413992604875) ENABLED START*/ //Add additional options here
  /*PROTECTED REGION END*/}
Stop::~Stop()
{
  /*PROTECTED REGION ID(dcon1413992604875) ENABLED START*/ //Add additional options here
  /*PROTECTED REGION END*/}
void Stop::run(void* msg)
{
  /*PROTECTED REGION ID(run1413992604875) ENABLED START*/ //Add additional options here
  CNPoint2D alloBallPos = wm->getBallPosition();
  cout << "BallPosition: (" << alloBallPos.x << " | " << alloBallPos.y << ")" << endl;

  /*PROTECTED REGION END*/}
void Stop::initialiseParameters()
{
  /*PROTECTED REGION ID(initialiseParameters1413992604875) ENABLED START*/ //Add additional options here
  /*PROTECTED REGION END*/}
/*PROTECTED REGION ID(methods1413992604875) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/} /* namespace alica */
