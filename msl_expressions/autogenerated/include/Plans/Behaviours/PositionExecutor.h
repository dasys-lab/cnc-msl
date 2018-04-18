#ifndef PositionExecutor_H_
#define PositionExecutor_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1438790362133) ENABLED START*/ // Add additional includes here
#include "MSLFootballField.h"
namespace msl {
	class MovementQuery;
}
namespace geometry {
	class CNPoint2D;
}

/*PROTECTED REGION END*/
namespace alica
{
class PositionExecutor : public DomainBehaviour
{
  public:
    PositionExecutor();
    virtual ~PositionExecutor();
    virtual void run(void *msg);
    /*PROTECTED REGION ID(pub1438790362133) ENABLED START*/ // Add additional public methods here
    /*PROTECTED REGION END*/
  protected:
    virtual void initialiseParameters();
    /*PROTECTED REGION ID(pro1438790362133) ENABLED START*/ // Add additional protected methods here
    /*PROTECTED REGION END*/
  private:
    /*PROTECTED REGION ID(prv1438790362133) ENABLED START*/ // Add additional private methods here
    static msl_actuator_msgs::MotionControl moveToPointFast(shared_ptr<geometry::CNPoint2D> egoTarget, shared_ptr<geometry::CNPoint2D> egoAlignPoint,
                                                            double snapDistance, shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints);

    void readConfigParameters();
    double fastCatchRadius;
    double slowCatchRadius;
    double alignTolerance;
    double ballDistanceEx;
    EntryPoint *receiverEp;
    shared_ptr<geometry::CNPoint2D> alloTarget;
    shared_ptr<msl::MovementQuery> query;

                                                            /*PROTECTED REGION END*/};
                                                            }
                                                            /* namespace alica */

#endif /* PositionExecutor_H_ */
