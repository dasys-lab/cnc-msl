/*
 * actuator.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Lukas Will
 */


#include "actuator.h"

// fuer Tests
//#include <thread>         // this_thread::sleep_for
//#include <chrono>         // chrono::seconds
//#include <stdio.h>		// File Open
//#include <unistd.h>		// File Open

using namespace std;
using namespace BlackLib;

mutex					mtx;
//condition_variable		cv, cv2;

uint8_t		th_count;

bool		th_activ = true;




void handleBallHandleControl(const msl_actuator_msgs::BallHandleCmd msg) {
	if (msg.enabled) {
		BH_right.setBallHandling(msg.rightMotor);
		BH_left.setBallHandling(msg.leftMotor);
	} else {
		BH_right.setBallHandling(0);
		BH_left.setBallHandling(0);
	}
}

void handleShovelSelectControl(const msl_actuator_msgs::ShovelSelectCmd msg) {
	shovel.last_ping = last_ping;

	// Schussauswahl (ggf Wert fuer Servoposition mit uebergeben lassen)
	if (msg.passing) {
		shovel.value = ShovelSelect_PASSING;
	} else {
		shovel.value = ShovelSelect_NORMAL;
	}
	shovel.enabled;
}

void handleMotionLight(const msl_actuator_msgs::MotionLight msg) {
	// LED vom Maussensor ansteuern
	adns3080.controlLED(msg.enable);
}


void controlBHLeft() {
	unique_lock<mutex> l_bhl(threw[0].mtx);
	while(th_activ) {
		threw[0].cv.wait(l_bhl, [&] { return !th_activ || threw[0].notify; }); // protection against spurious wake-ups
		if (!th_activ)
			return;

		BH_left.controlBallHandling();

		threw[0].notify = false;
		cv_main.cv.notify_all();
	}
}

void controlBHRight() {
	unique_lock<mutex> l_bhr(threw[1].mtx);
	while(th_activ) {
		threw[1].cv.wait(l_bhr, [&] { return !th_activ || threw[1].notify; }); // protection against spurious wake-ups
		if (!th_activ)
			return;

		BH_right.controlBallHandling();

		threw[1].notify = false;
		cv_main.cv.notify_all();
	}
}

void contolShovelSelect() {
	unique_lock<mutex> l_shovel(threw[2].mtx);
	while(th_activ) {
		threw[2].cv.wait(l_shovel, [&] { return !th_activ || threw[2].notify; }); // protection against spurious wake-ups
		if (!th_activ)
			return;

		if ((TIMEDIFFMS(time_now, shovel.last_ping) > ShovelSelect_TIMEOUT) && shovel.enabled) {
			shovel.enabled = false;
			ShovelSelect.setRunState(stop);
		}

		if (shovel.enabled) {
			if (ShovelSelect.getRunValue() == "0") {
				ShovelSelect.setRunState(run);
			}
			ShovelSelect.setSpaceRatioTime(shovel.value, microsecond);
		}

		threw[2].notify = false;
		cv_main.cv.notify_all();
	}
}

void getLightbarrier(ros::Publisher *hbiPub) {
	unique_lock<mutex> l_light(threw[3].mtx);
	while(th_activ) {
		threw[3].cv.wait(l_light, [&] { return !th_activ || threw[3].notify; }); // protection against spurious wake-ups
		if (!th_activ)
			return;

		msl_actuator_msgs::HaveBallInfo msg;
		uint16_t value = ADC_light.getNumericValue();

		if (value > LIGHTBARRIER_THRESHOLD) {
			msg.haveBall = true;
		} else {
			msg.haveBall = false;
		}
		hbiPub->publish(msg);

		threw[3].notify = false;
		cv_main.cv.notify_all();
	}
}

void getSwitches(ros::Publisher *bsPub, ros::Publisher *brtPub, ros::Publisher *vrtPub) {
	unique_lock<mutex> l_switches(threw[4].mtx);
	while(th_activ) {
		threw[4].cv.wait(l_switches, [&] { return !th_activ || threw[4].notify; }); // protection against spurious wake-ups
		if (!th_activ)
			return;

		msl_actuator_msgs::VisionRelocTrigger msg;
		std_msgs::Empty msg_empty;
		uint8_t bundle, power, vision;

		bundle = SW_Bundle.getNumericValue();
		vision = SW_Vision.getNumericValue();
		power = SW_Power.getNumericValue();

		msg.usePose = false;

		// TODO: Taster Entprellfunktion
		// TODO: Senden wenn nicht gedrueckt
		if (bundle == 1) {
			bsPub->publish(msg);
			brtPub->publish(msg_empty);
		}

		if (vision == 1) {
			vrtPub->publish(msg);
		}

		if (power == 1) {

		}

		threw[4].notify = false;
		cv_main.cv.notify_all();
	}
}

void getIMU(ros::Publisher *imuPub) {
	unique_lock<mutex> l_imu(threw[5].mtx);
	while(th_activ) {
		threw[5].cv.wait(l_imu, [&] { return !th_activ || threw[5].notify; }); // protection against spurious wake-ups
		if (!th_activ)
			return;

		lsm9ds0.updateData(time_now);
		lsm9ds0.sendData(time_now, imuPub);

		threw[5].notify = false;
		cv_main.cv.notify_all();
	}
}

void getOptical(ros::Publisher *mbcPub) {
	unique_lock<mutex> l_optical(threw[6].mtx);
	while(th_activ) {
		threw[6].cv.wait(l_optical, [&] { return !th_activ || threw[6].notify; }); // protection against spurious wake-ups
		if (!th_activ)
			return;

		adns3080.update_motion_burst(time_now);
		adns3080.send_motion_burst(time_now, mbcPub);

		threw[6].notify = false;
		cv_main.cv.notify_all();
	}
}

void exit_program(int sig) {
	ex = true;
	th_activ = false;
	cv_main.cv.notify_all();
	for (int i=0; i<6; i++)
		threw[i].cv.notify_all();
}




int main(int argc, char** argv) {
	// ROS Init
	ros::init(argc, argv, "ActuatorController");
	ros::NodeHandle node;
	ros::Rate loop_rate(30);		// in Hz

	ros::Subscriber sscSub = node.subscribe<msl_actuator_msgs::ShovelSelectCmd>("ShovelSelectControl", 25, handleShovelSelectControl);
	ros::Subscriber mlcSub = node.subscribe<msl_actuator_msgs::MotionLight>("CNActuator/MotionLight", 25, handleMotionLight);
	ros::Subscriber bhcSub = node.subscribe<msl_actuator_msgs::BallHandleCmd>("BallHandleControl", 25, handleBallHandleControl);

	ros::Publisher bsPub = node.advertise<msl_actuator_msgs::VisionRelocTrigger>("CNActuator/BundleStatus", 10);
	ros::Publisher brtPub = node.advertise<std_msgs::Empty>("CNActuator/BundleRestartTrigger", 10);
	ros::Publisher vrtPub = node.advertise<msl_actuator_msgs::VisionRelocTrigger>("CNActuator/VisionRelocTrigger", 10);
	ros::Publisher mbcPub = node.advertise<msl_actuator_msgs::MotionBurst>("CNActuator/MotionBurst", 10);
	ros::Publisher hbiPub = node.advertise<msl_actuator_msgs::HaveBallInfo>("HaveBallInfo", 10);
	//ros::Publisher imuPub = node.advertise<YYeigene msg bauenYY>("IMU", 10);

	thread th_controlBHRight(controlBHRight);
	thread th_controlBHLeft(controlBHLeft);
	thread th_controlShovel(contolShovelSelect);
	thread th_lightbarrier(getLightbarrier, &hbiPub);
	thread th_switches(getSwitches, &bsPub, &brtPub, &vrtPub);
	thread th_adns3080(getOptical, &mbcPub);
	//thread th_imu(getIMU, &imuPub);

	// Shovel Init
	ShovelSelect.setPeriodTime(ShovelSelect_PERIOD);	// in us - 20ms Periodendauer
	ShovelSelect.setSpaceRatioTime(1500000);			// in us - Werte zwischen 1ms und 2ms
	shovel.enabled = false;

	// I2C
	bool i2c = myI2C.open(ReadWrite);
	bool spi = mySpi.open(ReadWrite);
	bool imu = lsm9ds0.init();
	adns3080.reset();
	adns3080.adns_init();

	usleep(50000);

	(void) signal(SIGINT, exit_program);
	while(ros::ok() && !ex) {
		// Frequency: 30Hz - set with loop_rate()

		gettimeofday(&time_now, NULL);
		timeval vorher, mitte, nachher;

		// Thread Notify
		for (int i=0; i<7; i++) {
			threw[i].notify = true;
			threw[i].cv.notify_all();
		}

		// auf beenden aller Threads warten
		unique_lock<mutex> l_main(cv_main.mtx);
		cv_main.cv.wait(l_main, [&] { return !th_activ || (!threw[0].notify && !threw[1].notify && !threw[2].notify && !threw[3].notify && !threw[4].notify && !threw[6].notify); }); // protection against spurious wake-ups

		// MotionBurst

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
