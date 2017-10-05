#ifndef Pos2Defenders_H_
#define Pos2Defenders_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1444834678756) ENABLED START*/ //Add additional includes here
#include <msl_robot/robotmovement/MovementQuery.h>
#include <engine/RunningPlan.h>
#include <engine/Assignment.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <Ball.h>
#include <Robots.h>
#include <MSLWorldModel.h>
#include <MSLFootballField.h>

/*PROTECTED REGION END*/
namespace alica
{
    class Pos2Defenders : public DomainBehaviour
    {
    public:
        Pos2Defenders();
        virtual ~Pos2Defenders();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1444834678756) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1444834678756) ENABLED START*/ //Add additional protected methods here
        int keeperId;
        nonstd::optional<geometry::CNPositionEgo> keeperPos;
        msl::MovementQuery query;
        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1444834678756) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* Pos2Defenders_H_ */
