using namespace std;
#include "Plans/Behaviours/Joystick.h"

/*PROTECTED REGION ID(inccpp1421854975890) ENABLED START*/ //Add additional includes here
#include <msl_actuator_msgs/KickControl.h>
#include <msl_actuator_msgs/BallHandleCmd.h>
#include <msl_actuator_msgs/MotionControl.h>
#include <msl_actuator_msgs/KickControl.h>
#include <msl_actuator_msgs/ShovelSelectCmd.h>
#include "MSLWorldModel.h"
#include <RawSensorData.h>
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1421854975890) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	Joystick::Joystick() :
			DomainBehaviour("Joystick")
	{
		/*PROTECTED REGION ID(con1421854975890) ENABLED START*/ //Add additional options here
		pastTranslations = make_shared<std::vector<double>>();
		pastControlInput = make_shared<std::vector<double>>();
		/*PROTECTED REGION END*/
	}
	Joystick::~Joystick()
	{
		/*PROTECTED REGION ID(dcon1421854975890) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void Joystick::run(void* msg)
	{
		/*PROTECTED REGION ID(run1421854975890) ENABLED START*/ //Add additional options here
		auto joy = wm->rawSensorData->getJoystickCommand();
		if (!joy)
		{
			msl_actuator_msgs::MotionControl mc;
			send(mc);
			return;
		}

//		if (lastProcessedCmd == joy) // only process new commands from WM
//		{
//			msl_actuator_msgs::MotionControl mc;
//			send(mc);
//			return;
//		}
		cout << "joy->translation = " << joy->motion.translation << endl;
		//cout << "Joystick-Beh: " << *joy << endl;
		if (!std::isnan(joy->motion.translation) && !std::isnan(joy->motion.rotation) && !std::isnan(joy->motion.angle))
		{
			msl_actuator_msgs::MotionControl mc;

			if (pastTranslations->empty())
			{
				fillVector(pastTranslations, 4, mc.motion.translation);
			}
			if (pastControlInput->empty())
			{
				fillVector(pastControlInput, 3, joy->motion.translation);
			}

			// smooth driving stuff
//			if (this->smoothDrivingState)
//			{
			mc.motion.translation = ptController();
			cout << "Joystick: x = " << mc.motion.translation << endl;
//			} else
//			{
//				mc.motion = joy->motion;
//			}

			// acceleration memory
			// saving translation of the last 3 iterations
			updateVector(pastTranslations, mc.motion.translation);
			updateVector(pastControlInput, joy->motion.translation);
			send(mc);
		}
		else
		{
			msl_actuator_msgs::MotionControl mc;
			send(mc);
			//cout << "Joystick: Some Motion Value is NaN!" << endl;
		}

		if (joy->ballHandleState == msl_msgs::JoystickCommand::BALL_HANDLE_ON)
		{
			//cout << "Joy Ball handle on" << endl;
			msl_actuator_msgs::BallHandleCmd bhc;
			bhc.leftMotor = joy->ballHandleLeftMotor;
			bhc.rightMotor = joy->ballHandleRightMotor;
			send(bhc);
		}

		msl_actuator_msgs::ShovelSelectCmd ssc;

		if (joy->shovelIdx == 0)
		{
			ssc.passing = true;
		}
		else
		{
			ssc.passing = false;
		}
		send(ssc);

		if (joy->kick == true)
		{
			msl_actuator_msgs::KickControl kc;
			kc.power = joy->kickPower;
			kc.extension = joy->shovelIdx;
			kc.extTime = 1;
			kc.forceVoltage = false;
			send(kc);
		}

		lastProcessedCmd = joy;
		cout << "=========================" << endl;
		/*PROTECTED REGION END*/
	}
	void Joystick::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1421854975890) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1421854975890) ENABLED START*/ //Add additional methods here
	int Joystick::ptController()
	{
		// slope variable
		double a = 5;
		// changing point for slope
		double b = pow(a, 2);
		// sending frequency
		double TA = 1.0 / 30.0;


		double n1 = pow(a, 2) / (1 / pow(TA, 2) + (2 / TA) + pow(a, 2));
		double n2 = exp(-2 * a * TA) - exp(-a * TA) + exp(-a * TA) * TA;

		double d1 = (-(2 / pow(TA, 2)) - ((2 * a) / TA)) / (1 / pow(TA, 2) + 2 / TA + pow(a, 2));
		double d2 = (1 / pow(TA, 2)) / (1 / pow(TA, 2) + 2 / TA + pow(a, 2));
		double d3 = -2 * exp(-2 * a * TA) -2 * exp(-a * TA);
		double d4 = exp(-2 * a * TA);

		cout << "n1 = " << n1 << endl;
		cout << "n2 = " << n2 << endl;

		cout << "d1 = " << d1 << endl;
		cout << "d2 = " << d2 << endl;
		cout << "d3 = " << d3 << endl;
		cout << "d4 = " << d4 << endl;

		return n1 * pastControlInput->at(0) + n2 * pastControlInput->at(1)
				- d1 * pastTranslations->at(0) - d2 * pastTranslations->at(1) - d3 * pastTranslations->at(2)
				- d4 * pastTranslations->at(3);
	}

	void Joystick::fillVector(shared_ptr<std::vector<double>> vector, int size, int translation)
	{
		for (int i = 0; i < size; i++)
		{
			if (i == size - 1)
			{
				vector->insert(vector->begin(), translation);
				return;
			}
			vector->insert(vector->begin(), 0);
		}
	}

	void Joystick::updateVector(shared_ptr<std::vector<double>> vector, int translation)
	{
		vector->insert(vector->begin(), translation);
		vector->pop_back();
	}
/*PROTECTED REGION END*/
} /* namespace alica */
