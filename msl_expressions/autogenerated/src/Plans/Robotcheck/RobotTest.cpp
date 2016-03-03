using namespace std;
#include "Plans/Robotcheck/RobotTest.h"

/*PROTECTED REGION ID(inccpp1456756113767) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1456756113767) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	RobotTest::RobotTest() :
			DomainBehaviour("RobotTest")
	{
		/*PROTECTED REGION ID(con1456756113767) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	RobotTest::~RobotTest()
	{
		/*PROTECTED REGION ID(dcon1456756113767) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void RobotTest::run(void* msg)
	{
		/*PROTECTED REGION ID(run1456756113767) ENABLED START*/ //Add additional options here
		auto me = wm->rawSensorData.getOwnPositionVision();

		// testing motion ================================================================

		if (driveForward)
		{
			driveForward = translationRotationRobot(-300, true, 3000);
			if (!driveForward)
			{
				cout << "testing drive back ..." << endl;
				driveBack = true;
			}
		}
		if (driveBack)
		{
			driveBack = translationRotationRobot(300, true, 3000);
			if (!driveBack)
			{
				cout << "testing rotate right ..." << endl;
				rotateForward = true;
			}
		}
		if (rotateForward)
		{
			rotateForward = translationRotationRobot(-2, false, 3000);
			if (!rotateForward)
			{
				cout << "testing rotate left ..." << endl;
				rotateBack = true;
			}
		}
		if (rotateBack)
		{
			rotateBack = translationRotationRobot(2, false, 3000);
			if (!rotateBack)
			{
				cout << "testing rotate actuator forward ..." << endl;
				actuatorForward = true;
			}
		}

		// testing actuator ==============================================================

		if (actuatorForward)
		{
			actuatorForward = actuatorRobot(3000, 30);
			if (!actuatorForward)
			{
				cout << "testing rotate actuator back ..." << endl;
				actuatorBack = true;
			}
		}
		if (actuatorBack)
		{
			actuatorBack = actuatorRobot(3000, -30);
			if (!actuatorBack)
			{
				cout << "testing light barrier ..." << endl;
				lightBarrier = true;
			}
		}

		// light barrier ==============================================================

		if (lightBarrier)
		{
			lightBarrier = lightBarrierRobot();
			if (!lightBarrier)
			{
				cout << "testing optical flow ..." << endl;
				opticalFlow = true;
			}
		}

		// optical flow ===============================================================

		if (opticalFlow)
		{
			opticalFlow = opticalFlowRobot();
			if (!opticalFlow)
			{
				cout << "testing IMU ..." << endl;
				imu = true;
			}
		}

		// IMU ========================================================================

		if (imu)
		{
			imu = imuRobot();
			if (!imu)
			{
				cout << "testing shovelSelectLow ..." << endl;
				shovelSelectLow = true;
			}
		}

		// shovel select ==============================================================

		if (shovelSelectLow)
		{
			shovelSelectLow = shovelSelectRobot(true, 3000);
			if (!shovelSelectLow)
			{
				cout << "testing shovelSelectHigh ..." << endl;
				shovelSelectHigh = true;
			}
		}

		if (shovelSelectHigh)
		{
			shovelSelectHigh = shovelSelectRobot(false, 3000);
			if (!shovelSelectHigh)
			{
				cout << "kicking = " << kickPower << endl;
				kicker = true;
			}
		}

		// testing kicker ================================================================

		if (kicker)
		{
			kicker = kickerRobot(kickPower);
		}

		if (!driveForward && !driveBack && !rotateBack && !rotateForward && !kicker && !actuatorForward && !actuatorBack
				&& !opticalFlow && !imu && !shovelSelectLow && !shovelSelectHigh && !lightBarrier)
		{
			cout << "finished testing" << endl;
//		cout
//				<< "This robot check behavior was presented by very fast and often working Michael Gottesleben and Lukas Will!"
//				<< endl;
			cout << "Have fun bitches i'm out!\n" << endl;
//		printGlasses();
			this->success = true;
		}
		/*PROTECTED REGION END*/
	}
	void RobotTest::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1456756113767) ENABLED START*/ //Add additional options here
		move = 0;

		driveForward = true;
		driveBack = false;
		rotateBack = false;
		rotateForward = false;
		kicker = false;
		actuatorForward = false;
		actuatorBack = false;
		opticalFlow = false;
		imu = false;
		shovelSelectLow = false;
		shovelSelectHigh = false;
		lightBarrier = false;

		kickPower = 300;

		if (driveForward)
		{
			cout << "\nstart testing motion" << endl;
			cout << "testing drive forward ..." << endl;
		} else
		{
			cout << "\nstart testing ... \n"
					"test is manually configured!" << endl;
		}

		/*PROTECTED REGION END*/
	}
	/*PROTECTED REGION ID(methods1456756113767) ENABLED START*/ //Add additional methods here
	bool RobotTest::translationRotationRobot(int movement, bool trans, int duration)
	{
		msl_actuator_msgs::MotionControl mc;
		if (trans)
		{
			if (move < (30 * duration) / 1000 && trans)
			{
				mc.motion.translation = movement;
			}
			else
			{
				move = 0;
				return false;
			}
		}
		else
		{
			if (move < (30 * duration) / 1000 && !trans)
			{
				mc.motion.rotation = movement;
			}
			else
			{
				move = 0;
				return false;
			}
		}
		move++;
		send(mc);

		return true;
	}

	bool RobotTest::kickerRobot(int power)
	{
		msl_actuator_msgs::KickControl kc;
		kc.enabled = true;
		if (power > 0)
		{
			kc.power = power;
			send(kc);
			return false;

		}
		else
		{
			cerr << "Kick power may not be negative!" << endl;
			kc.power = 0;
		}
		return true;
	}

	bool RobotTest::actuatorRobot(int duration, int power)
	{
		msl_actuator_msgs::BallHandleCmd bhc;
		if (move < (30 * duration) / 1000)
		{
			if (power < 100 && power > -100)
			{
				bhc.leftMotor = power;
				bhc.rightMotor = power;
			}
			else
			{
				bhc.leftMotor = 0;
				bhc.rightMotor = 0;
				cerr << "Rotation speed may only be in range of -100 to 100!" << endl;
			}
		}
		else
		{
			move = 0;
			return false;
		}

		move++;
		bhc.enabled = true;
		send(bhc);
		return true;
	}

	bool RobotTest::lightBarrierRobot()
	{
		auto lbi = wm->rawSensorData.getLightBarrier();

		if (lbi)
		{
			bool static lb_old = *lbi;
			if (lb_old != *lbi)
			{
				lb_old = *lbi;
				cout << "toggle light barrier!" << endl;
				move++;
			}
			if (move > 5)
			{
				move = 0;
				return false;
			}
		}
		else
		{
			std::cout << "light barrier = nullPtr " << std::endl;
			return false;
		}
		return true;
	}

	bool RobotTest::opticalFlowRobot()
	{
		auto of = wm->rawSensorData.getOpticalFlow();
		if (of != nullptr)
		{
			cout << "receive data from optical flow!" << endl;
			return false;
		}
		else
		{
			cerr << "No data from optical flow!" << endl;
			return false;
		}
		return true;
	}

	bool RobotTest::imuRobot()
	{
		cout << "IMU currently not included in world model" << endl;
		return false;
	}

	bool RobotTest::shovelSelectRobot(bool pass, int duration)
	{
		msl_actuator_msgs::ShovelSelectCmd sc;
		if (move < (30 * duration) / 1000)
		{
			sc.passing = pass;
			send(sc);
		}
		else
		{
			move = 0;
			return false;
		}
		move++;
		return true;
	}

	void RobotTest::printGlasses()
	{
		cout << "   IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII\n "
				<< "IIII              IIIIIII   III   IIIIIIIIIIIII   IIIIII   III   IIIIIIIIIIIII\n"
				<< "                     IIIIIII   III   IIIIIIIIII      IIIIII   III   IIIIIIII\n"
				<< "                        IIIIIII   III   IIII            IIIIII   III   III\n"
				<< "                          IIIIIIIIIIIIIII                IIIIIIIIIIIIIII\n" << endl;
	}

/*PROTECTED REGION END*/
} /* namespace alica */
