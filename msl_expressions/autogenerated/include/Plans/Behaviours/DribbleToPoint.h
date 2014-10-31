#ifndef DribbleToPoint_H_
#define DribbleToPoint_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1414752367688) ENABLED START*/ //Add additional includes here
#include "MSLWorldModel.h"
/*PROTECTED REGION END*/
namespace alica
{
class DribbleToPoint : public DomainBehaviour
{
public:
  DribbleToPoint();
  virtual ~DribbleToPoint();
  virtual void run(void* msg);
  /*PROTECTED REGION ID(pub1414752367688) ENABLED START*/ //Add additional public methods here
  bool haveBall();
  void getBall();
  pair<double, double> allo2Ego(pair<double, double>& p, tuple<double, double, double>& ownPos);
  /*PROTECTED REGION END*/
protected:
  virtual void initialiseParameters();
  /*PROTECTED REGION ID(pro1414752367688) ENABLED START*/ //Add additional protected methods here
  /*PROTECTED REGION END*/
private:
  /*PROTECTED REGION ID(prv1414752367688) ENABLED START*/ //Add additional private methods here
  /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DribbleToPoint_H_ */
