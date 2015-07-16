#ifndef DribbleToAttackPoint_H_
#define DribbleToAttackPoint_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1436855838589) ENABLED START*/ //Add additional includes here
#include "MSLFootballField.h"
#include "SystemConfig.h"
#include <ros/ros.h>
/*PROTECTED REGION END*/
namespace alica
{
    class DribbleToAttackPoint : public DomainBehaviour
    {
    public:
        DribbleToAttackPoint();
        virtual ~DribbleToAttackPoint();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1436855838589) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1436855838589) ENABLED START*/ //Add additional protected methods here
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1436855838589) ENABLED START*/ //Add additional private methods here
        shared_ptr<geometry::CNPoint2D> alloTargetPoint;
        shared_ptr<geometry::CNPoint2D> egoTargetPoint;
        msl::MSLFootballField* field;
        supplementary::SystemConfig* sc;
        int wheelSpeed;
        shared_ptr<geometry::CNPoint2D> lastClosesOpp;
        ros::Publisher voroniPub;
        ros::NodeHandle n;
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* DribbleToAttackPoint_H_ */