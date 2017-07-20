#ifndef Joystick_H_
#define Joystick_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1421854975890) ENABLED START*/ //Add additional includes here
#include <msl_msgs/JoystickCommand.h>

namespace msl_msgs
{
	ROS_DECLARE_MESSAGE(JoystickCommand)
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
		shared_ptr<std::vector<double>> pastTranslations;
		shared_ptr<std::vector<double>> pastControlInput;

		int ptController();

		void fillVector(shared_ptr<std::vector<double>> vector, int size, int translation);
		void updateVector(shared_ptr<std::vector<double>> vector, int translation);

		/*PROTECTED REGION END*/
	private:
		/*PROTECTED REGION ID(prv1421854975890) ENABLED START*/ //Add additional private methods here
		/*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* Joystick_H_ */
