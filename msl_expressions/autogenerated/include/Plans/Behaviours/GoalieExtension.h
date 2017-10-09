#ifndef GoalieExtension_H_
#define GoalieExtension_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1459249216387) ENABLED START*/ //Add additional includes here
#include "DateTime.h"
#include <msl_actuator_msgs/MotionControl.h>
#include <msl_actuator_msgs/KickControl.h>
#include <cnc_geometry/CNPointAllo.h>
#include <nonstd/optional.hpp>
namespace alica
{
    /**
     * use at your own risk!!
     */
    class ExperimentalRingbuffer
    {
    protected:
        int indexMax;
        int index;
        vector<geometry::CNPointAllo> buffer;
        vector<double> weights;

    public:
        ExperimentalRingbuffer(int size);
        ~ExperimentalRingbuffer();

        geometry::CNPointAllo getAvgPoint(int count);

        void addPoint(geometry::CNPointAllo p);

        void overWrite(geometry::CNPointAllo p);
        void addPoint(geometry::CNPointAllo p, double g);

        void overWrite(geometry::CNPointAllo p, double g);
    };


}
/*PROTECTED REGION END*/
namespace alica
{
    class GoalieExtension : public DomainBehaviour
    {
    public:
        GoalieExtension();
        virtual ~GoalieExtension();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1459249216387) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1459249216387) ENABLED START*/ //Add additional protected methods here
        long KICKER_WAIT_TIME;
        long lastKickerTime;
        bool useExt1, useExt2, useExt3, useKicker;
        msl_actuator_msgs::KickControl km;
        ExperimentalRingbuffer* ballGoalProjection;
        ExperimentalRingbuffer* ballVelocity;
        long ballInAirTimestamp;
        msl_actuator_msgs::MotionControl bm_last;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1459249216387) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* GoalieExtension_H_ */
