#ifndef PassIntoFreeZone_H_
#define PassIntoFreeZone_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1508951632953) ENABLED START*/ // Add additional includes here
#include <engine/constraintmodul/Query.h>
#include <msl_robot/robotmovement/MovementQuery.h>
/*PROTECTED REGION END*/
namespace alica
{
class PassIntoFreeZone : public DomainBehaviour
{
  public:
    PassIntoFreeZone();
    virtual ~PassIntoFreeZone();
    virtual void run(void *msg);
    /*PROTECTED REGION ID(pub1508951632953) ENABLED START*/ // Add additional public methods here
                                                            /*PROTECTED REGION END*/
  protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1508951632953) ENABLED START*/ // Add additional protected methods here
    vector<double> result;
    shared_ptr<alica::Query> query;
    shared_ptr<msl::MovementQuery> mQuery;
    /*PROTECTED REGION END*/
  private:
    /*PROTECTED REGION ID(prv1508951632953) ENABLED START*/ // Add additional private methods here
        /*PROTECTED REGION END*/};
        } /* namespace alica */

#endif /* PassIntoFreeZone_H_ */
