#ifndef DriveToBall_H_
#define DriveToBall_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1447863493623) ENABLED START*/ //Add additional includes here
#include "container/CNPoint2D.h"
#include "container/CNPosition.h"

using namespace msl;
/*PROTECTED REGION END*/
namespace alica
{
	class DriveToBall : public DomainBehaviour
	{
	public:
		DriveToBall();
		virtual ~DriveToBall();
		virtual void run(void* msg);
		/*PROTECTED REGION ID(pub1447863493623) ENABLED START*/ //Add additional public methods here
		/*PROTECTED REGION END*/
	protected:
		virtual void initialiseParameters();
		/*PROTECTED REGION ID(pro1447863493623) ENABLED START*/ //Add additional protected methods here
		/*PROTECTED REGION END*/
	private:
		/*PROTECTED REGION ID(prv1447863493623) ENABLED START*/ //Add additional private methods here
    	// TODO: DELETE THIS BECAUSE NOT USED IN GoalieDefault PLAN!!!
		/*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DriveToBall_H_ */
