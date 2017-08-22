#ifndef Joystick_H_
#define Joystick_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1421854975890) ENABLED START*/ //Add additional includes here
#include <msl_msgs/JoystickCommand.h>
#include <queue>
#include <valarray>

namespace msl_msgs
{
    ROS_DECLARE_MESSAGE (JoystickCommand)
}
/*PROTECTED REGION END*/
namespace alica
{
    class Joystick : public DomainBehaviour
    {
    public:
        Joystick();
        virtual ~Joystick();
        virtual void run(void* msg);
        /*PROTECTED REGION ID(pub1421854975890) ENABLED START*/ //Add additional public methods here
        /*PROTECTED REGION END*/
    protected:
        virtual void initialiseParameters();
        /*PROTECTED REGION ID(pro1421854975890) ENABLED START*/ //Add additional protected methods here
        shared_ptr<msl_msgs::JoystickCommand> lastProcessedCmd;
        std::queue<std::valarray<double>> pastTranslations;
        std::queue<std::valarray<double>> pastControlInput;
        double init[3] = {0.0, 0.0, 0.0};

        std::valarray<double> ptController();
        int lastJump;

        /*PROTECTED REGION END*/
    private:
        /*PROTECTED REGION ID(prv1421854975890) ENABLED START*/ //Add additional private methods here
        /*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* Joystick_H_ */
