#ifndef PositionReceiver_H_
#define PositionReceiver_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1439379316897) ENABLED START*/ //Add additional includes here
#include "msl_robot/robotmovement/MovementQuery.h"
using namespace std;
/*PROTECTED REGION END*/
namespace alica
{
    class PositionReceiver : public DomainBehaviour
    {
    public:
        PositionReceiver();
        virtual ~PositionReceiver();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1439379316897) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1439379316897) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1439379316897) ENABLED START*/ //Add additional private methods here
        static msl_actuator_msgs::MotionControl moveToPointFast(shared_ptr<geometry::CNPoint2D> egoTarget,
                                                                shared_ptr<geometry::CNPoint2D> egoAlignPoint,
                                                                double snapDistance,
                                                                shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints);

                                                            double fastCatchRadius;
                                                            double slowCatchRadius;
                                                            double alignTolerance;
                                                            double ballDistanceRec;
                                                            shared_ptr<msl::MovementQuery> query;
                                                            /*PROTECTED REGION END*/};
                                                    }
                                                    /* namespace alica */

#endif /* PositionReceiver_H_ */
