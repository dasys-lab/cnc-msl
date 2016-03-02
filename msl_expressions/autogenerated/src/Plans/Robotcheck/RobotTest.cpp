using namespace std;
#include "Plans/Robotcheck/RobotTest.h"

/*PROTECTED REGION ID(inccpp1456756113767) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
/*PROTECTED REGION END*/
namespace alica {
/*PROTECTED REGION ID(staticVars1456756113767) ENABLED START*/ //initialise static variables here
/*PROTECTED REGION END*/
RobotTest::RobotTest() :
		DomainBehaviour("RobotTest") {
	/*PROTECTED REGION ID(con1456756113767) ENABLED START*/ //Add additional options here
	/*PROTECTED REGION END*/
}
RobotTest::~RobotTest() {
	/*PROTECTED REGION ID(dcon1456756113767) ENABLED START*/ //Add additional options here
	/*PROTECTED REGION END*/
}
void RobotTest::run(void* msg) {
	/*PROTECTED REGION ID(run1456756113767) ENABLED START*/ //Add additional options here
	auto me = wm->rawSensorData.getOwnPositionVision();

	// testing motion ================================================================

	if (driveForward) {
		cout << "drive forward!" << endl;
		driveForward = translationRotationRobot(300, true, 3000);
		if (!driveForward) {
			driveBack = true;
		}
	}
	if (driveBack) {
		cout << "drive back!" << endl;
		driveBack = translationRotationRobot(-300, true, 3000);
		if (!driveBack) {
			rotateForward = true;
		}
	}
	if (rotateForward) {
		cout << "rotate right!" << endl;
		rotateForward = translationRotationRobot(2, false, 3000);
		if (!rotateForward) {
			rotateBack = true;
		}
	}
	if (rotateBack) {
		cout << "rotate left!" << endl;
		rotateBack = translationRotationRobot(-2, false, 3000);
		if (!rotateBack) {
			actuatorForward = true;
		}
	}

	// testing actuator ==============================================================

	if (actuatorForward) {
		cout << " rotate actuator forward" << endl;
		actuatorForward = actuatorRobot(3000, 30);
		if (!actuatorForward) {
			actuatorBack = true;
		}
	}
	if (actuatorBack) {
		cout << " rotate actuator back" << endl;
		actuatorBack = actuatorRobot(3000, -30);
		if (!actuatorBack) {
			lightBarrier = true;
		}
	}

	// light barrier ==============================================================

	if (lightBarrier) {
		cout << "light barrier  anfang= " << lightBarrier << endl;
		cout << "testing light barrier" << endl;
		lightBarrier = lightBarrierRobot();
		cout << "light barrier after method= " << lightBarrier << endl;
		if (!lightBarrier) {
			opticalFlow = true;
		}
	}

	// optical flow ===============================================================

	if (opticalFlow) {
		cout << "testing optical flow" << endl;
		opticalFlow = opticalFlowRobot();
		if (!opticalFlow) {
			imu = true;
		}
	}

	// IMU ========================================================================

	if (imu) {
		cout << "testing IMU" << endl;
		imu = imuRobot();
		if (!imu) {
			shovelSelectLow = true;
		}
	}

	// shovel select ==============================================================

	if (shovelSelectLow) {
		cout << "testing shovelSelectLow" << endl;
		shovelSelectLow = shovelSelectRobot(true, 3000);
		if (!shovelSelectLow) {
			shovelSelectHigh = true;
		}
	}

	if (shovelSelectHigh) {
		cout << "testing shovelSelectHigh" << endl;
		shovelSelectHigh = shovelSelectRobot(false, 3000);
		if (!shovelSelectHigh) {
			kicker = true;
		}
	}

	// testing kicker ================================================================

	if (kicker) {
		cout << "kicking = 300" << endl;
		kicker = kickerRobot(300);
//		if (!kicker) {
//			actuatorForward = true;
//		}
	}

	if (!driveForward && !driveBack && !rotateBack && !rotateForward && !kicker
			&& !actuatorForward && !actuatorBack && !opticalFlow && !imu
			&& !shovelSelectLow && !shovelSelectHigh && !lightBarrier) {
		cout << "finished testing" << endl;
		cout
				<< "This robot check behavior was presented by very fast and often working Michael Gottesleben and Lukas Will!"
				<< endl;
		cout << "Have fun bitches i'm out!\n" << endl;
		printGlasses();
		this->success = true;
	}
	/*PROTECTED REGION END*/
}
void RobotTest::initialiseParameters() {
	/*PROTECTED REGION ID(initialiseParameters1456756113767) ENABLED START*/ //Add additional options here
	move = 0;

	driveForward = false;
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
	lightBarrier = true;

	/*PROTECTED REGION END*/
}
/*PROTECTED REGION ID(methods1456756113767) ENABLED START*/ //Add additional methods here
bool RobotTest::translationRotationRobot(int movement, bool trans,
		int duration) {
	msl_actuator_msgs::MotionControl mc;
	cout << "move = " << move << endl;
	if (trans) {
		if (move < (30 * duration) / 1000 && trans) {
			mc.motion.translation = movement;
		} else {
			move = 0;
			return false;
		}
	} else {
		if (move < (30 * duration) / 1000 && !trans) {
			mc.motion.rotation = movement;
		} else {
			move = 0;
			return false;
		}
	}
	move++;
	send(mc);

	return true;
}

bool RobotTest::kickerRobot(int power) {
	msl_actuator_msgs::KickControl kc;
	kc.enabled = true;
	if (power > 0) {
		kc.power = power;
		send(kc);
		return false;

	} else {
		cerr << "Kick power may not be negative!" << endl;
		kc.power = 0;
	}
	return true;
}

bool RobotTest::actuatorRobot(int duration, int power) {
	msl_actuator_msgs::BallHandleCmd bhc;
	if (move < (30 * duration) / 1000) {
		if (power < 100 && power > -100) {
			bhc.leftMotor = power;
			bhc.rightMotor = power;
		} else {
			bhc.leftMotor = 0;
			bhc.rightMotor = 0;
			cerr << "Rotation speed may only be in range of -100 to 100!"
					<< endl;
		}
	} else {
		move = 0;
		return false;
	}

	move++;
	bhc.enabled = true;
	send(bhc);
	return true;
}

bool RobotTest::lightBarrierRobot() {
	auto lbi = wm->rawSensorData.getLightBarrier();

//	if (lbi != nullptr) {
//		auto static lb_old = *lbi;
//		auto lb_new = *lbi;
//
//		if (lb_old != lb_new) {
//			lb_old = lb_new;
//			cout << "light barrier = " << lb_old << endl;
//			move++;
//		}
//
//		if (move > 5) {
//			move = 0;
//			cout << "light barrier is working!" << endl;
//			return false;
//		}
//	} else {
//		cerr << "no data from light barrier!" << endl;
//		return false;
//	}

	if (lbi) {
		bool static lb_old = *lbi;
		cout << "light barrier 1 = " << lightBarrier << endl;
		if (lb_old != *lbi) {
			lb_old = *lbi;
			cout << "light barrier = " << lb_old << endl;
			move++;
		}
		cout << "light barrier 2 = " << lightBarrier << endl;
		if (move > 5) {
			move = 0;
			cout << "light barrier is working!" << endl;
			return false;
		}
		cout << "light barrier = 3 " << lightBarrier << endl;
		if (*lbi) {
			std::cout << "Hab den Ball :)" << std::endl;
		} else {
			std::cout << "Hab ihn nicht. :(" << std::endl;
		}
		cout << "light barrier = 4 " << lightBarrier << endl;
	} else {
		std::cout << "NullPtr :(" << std::endl;
		return false;
	}
	cout << "light barrier 5 = " << lightBarrier << endl;
	return true;
}

bool RobotTest::opticalFlowRobot() {
	// for testing
	return false;
	auto of = wm->rawSensorData.getOpticalFlow();
	if (of != nullptr) {
		return false;
	} else {
		cerr << "No data from optical flow!" << endl;
		return false;
	}
	return true;
}

bool RobotTest::imuRobot() {
	cout << "IMU currently not included in world model" << endl;
	return false;
}

bool RobotTest::shovelSelectRobot(bool pass, int duration) {
	msl_actuator_msgs::ShovelSelectCmd sc;
	if (move < (30 * duration) / 1000) {
		sc.passing = pass;
		send(sc);
	} else {
		move = 0;
		return false;
	}
	move++;
	return true;
}

void RobotTest::printGlasses() {
	cout
			<< "   IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII\n "
			<< "IIII              IIIIIII   III   IIIIIIIIIIIII   IIIIII   III   IIIIIIIIIIIII\n"
			<< "                     IIIIIII   III   IIIIIIIIII      IIIIII   III   IIIIIIII\n"
			<< "                        IIIIIII   III   IIII            IIIIII   III   III\n"
			<< "                          IIIIIIIIIIIIIII                IIIIIIIIIIIIIII\n"
			<< endl;
}
/*PROTECTED REGION END*/
} /* namespace alica */

