#ifndef GetBall_H_
#define GetBall_H_

#include "DomainBehaviour.h"
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"

using namespace msl;

/*PROTECTED REGION ID(inc1414828300860) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
class GetBall : public DomainBehaviour
{
public:
  GetBall();
  virtual ~GetBall();
  virtual void run(void* msg);
  /*PROTECTED REGION ID(pub1414828300860) ENABLED START*/ //Add additional public methods here
  /*PROTECTED REGION END*/
protected:
  virtual void initialiseParameters();
  /*PROTECTED REGION ID(pro1414828300860) ENABLED START*/ //Add additional protected methods here
  /*PROTECTED REGION END*/
private:
  /*PROTECTED REGION ID(prv1414828300860) ENABLED START*/ //Add additional private methods here
  /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* GetBall_H_ */
