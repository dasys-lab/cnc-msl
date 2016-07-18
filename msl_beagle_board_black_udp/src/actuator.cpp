/*
 * actuator.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Lukas Will
 */


#include "actuator.h"



#include <string>
#include <sstream>
#include <iostream>
#include <unistd.h>
#include "ros/ros.h"
#include <ros/transport_hints.h>
#include <stdio.h>


#include <SystemConfig.h>
#include <Configuration.h>
#include <exception>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <usbcanconnection.h>

#include "msl_actuator_msgs/BallHandleCmd.h"
#include "msl_actuator_msgs/BallHandleMode.h"
#include "msl_actuator_msgs/ShovelSelectCmd.h"
#include "msl_actuator_msgs/MotionLight.h"
#include "process_manager/ProcessCommand.h"
#include "msl_actuator_msgs/VisionRelocTrigger.h"
#include "msl_actuator_msgs/MotionBurst.h"
#include "std_msgs/Bool.h"
#include "msl_actuator_msgs/CanMsg.h"
#include "msl_actuator_msgs/IMUData.h"
#include "msl_actuator_msgs/RawOdometryInfo.h"
#include "../include/CanHandler.h"

using boost::asio::ip::udp;

std::string ownRosName;
udp::socket* insocket;
udp::endpoint otherEndPoint;
udp::endpoint destEndPoint;
boost::asio::ip::address multiCastAddress;
boost::asio::io_service io_service;
void handleUdpPacket(const boost::system::error_code& error,   std::size_t bytes_transferred);
void listenForPacket();






using namespace std;
using namespace BlackLib;

mutex		mtx;
uint8_t		th_count;
bool		th_activ = true;

BallHandle	ballHandle;

// Can hack
CanHandler canHandler;

void handleBallHandleControl(const msl_actuator_msgs::BallHandleCmd msg) {
	const msl_actuator_msgs::BallHandleMode mode;

	ballHandle.ping();
	if (ballHandle.getMode() == mode.REMOTE_CONTROL) {
		ballHandle.setBallHandling(msg.leftMotor, msg.rightMotor);
	}
}

void handleBallHandleMode(const msl_actuator_msgs::BallHandleMode msg) {
	ballHandle.ping();
	ballHandle.setMode(msg.mode);
}

void handleShovelSelectControl(const msl_actuator_msgs::ShovelSelectCmd msg) {
	try {
		shovel.setShovel(msg.passing, time_now);
	} catch (exception &e) {
		cout << "ShovelSelect: " << e.what() << endl;
	}
}

void handleMotionLight(const msl_actuator_msgs::MotionLight msg) {
	// LED vom Maussensor ansteuern
	try {
		adns3080.controlLED(msg.enable);
	} catch (exception &e) {
		cout << "MotionLight: " << e.what() << endl;
	}
}

void handleRawOdometryInfo(const msl_actuator_msgs::RawOdometryInfo msg) {
	ballHandle.setOdometryData(msg.motion.angle, msg.motion.translation);
}

void handleCanSub(const msl_actuator_msgs::CanMsg &msg) {
	// Nachricht an ueber can verschicken
	canHandler.sendCanMsg(msg);
}


void onRosBallHandleCmd1334345447(msl_actuator_msgs::BallHandleCmd& message) {
uint8_t* buffer = NULL;
	try{
		uint32_t serial_size = ros::serialization::serializationLength(message);
		buffer = new uint8_t[serial_size+sizeof(uint32_t)];
		ros::serialization::OStream stream(buffer+sizeof(uint32_t), serial_size);
		*((uint32_t*)buffer) = 1334345447u;
		ros::serialization::serialize(stream, message);
		// write message to UDP
		insocket->send_to(boost::asio::buffer((void*)buffer,serial_size+sizeof(uint32_t)),destEndPoint);
	} catch(std::exception& e) {
		ROS_ERROR_STREAM_THROTTLE(2,"Exception while sending UDP message:"<<e.what()<< " Discarding message!");
	}
	if(buffer!=NULL) delete[] buffer;
}
void onRosBallHandleMode297244167(msl_actuator_msgs::BallHandleMode& message) {
uint8_t* buffer = NULL;
	try{
		uint32_t serial_size = ros::serialization::serializationLength(message);
		buffer = new uint8_t[serial_size+sizeof(uint32_t)];
		ros::serialization::OStream stream(buffer+sizeof(uint32_t), serial_size);
		*((uint32_t*)buffer) = 297244167u;
		ros::serialization::serialize(stream, message);
		// write message to UDP
		insocket->send_to(boost::asio::buffer((void*)buffer,serial_size+sizeof(uint32_t)),destEndPoint);
	} catch(std::exception& e) {
		ROS_ERROR_STREAM_THROTTLE(2,"Exception while sending UDP message:"<<e.what()<< " Discarding message!");
	}
	if(buffer!=NULL) delete[] buffer;
}
void onRosShovelSelectCmd1418208429(msl_actuator_msgs::ShovelSelectCmd& message) {
uint8_t* buffer = NULL;
	try{
		uint32_t serial_size = ros::serialization::serializationLength(message);
		buffer = new uint8_t[serial_size+sizeof(uint32_t)];
		ros::serialization::OStream stream(buffer+sizeof(uint32_t), serial_size);
		*((uint32_t*)buffer) = 1418208429u;
		ros::serialization::serialize(stream, message);
		// write message to UDP
		insocket->send_to(boost::asio::buffer((void*)buffer,serial_size+sizeof(uint32_t)),destEndPoint);
	} catch(std::exception& e) {
		ROS_ERROR_STREAM_THROTTLE(2,"Exception while sending UDP message:"<<e.what()<< " Discarding message!");
	}
	if(buffer!=NULL) delete[] buffer;
}
void onRosMotionLight2056271736(msl_actuator_msgs::MotionLight& message) {
uint8_t* buffer = NULL;
	try{
		uint32_t serial_size = ros::serialization::serializationLength(message);
		buffer = new uint8_t[serial_size+sizeof(uint32_t)];
		ros::serialization::OStream stream(buffer+sizeof(uint32_t), serial_size);
		*((uint32_t*)buffer) = 2056271736u;
		ros::serialization::serialize(stream, message);
		// write message to UDP
		insocket->send_to(boost::asio::buffer((void*)buffer,serial_size+sizeof(uint32_t)),destEndPoint);
	} catch(std::exception& e) {
		ROS_ERROR_STREAM_THROTTLE(2,"Exception while sending UDP message:"<<e.what()<< " Discarding message!");
	}
	if(buffer!=NULL) delete[] buffer;
}
void onRosProcessCommand554624761(process_manager::ProcessCommand& message) {
uint8_t* buffer = NULL;
	try{
		uint32_t serial_size = ros::serialization::serializationLength(message);
		buffer = new uint8_t[serial_size+sizeof(uint32_t)];
		ros::serialization::OStream stream(buffer+sizeof(uint32_t), serial_size);
		*((uint32_t*)buffer) = 3108117629u;
		ros::serialization::serialize(stream, message);
		// write message to UDP
		insocket->send_to(boost::asio::buffer((void*)buffer,serial_size+sizeof(uint32_t)),destEndPoint);
	} catch(std::exception& e) {
		ROS_ERROR_STREAM_THROTTLE(2,"Exception while sending UDP message:"<<e.what()<< " Discarding message!");
	}
	if(buffer!=NULL) delete[] buffer;
}
void onRosVisionRelocTrigger2772566283(msl_actuator_msgs::VisionRelocTrigger& message) {
uint8_t* buffer = NULL;
	try{
		uint32_t serial_size = ros::serialization::serializationLength(message);
		buffer = new uint8_t[serial_size+sizeof(uint32_t)];
		ros::serialization::OStream stream(buffer+sizeof(uint32_t), serial_size);
		*((uint32_t*)buffer) = 2772566283u;
		ros::serialization::serialize(stream, message);
		// write message to UDP
		insocket->send_to(boost::asio::buffer((void*)buffer,serial_size+sizeof(uint32_t)),destEndPoint);
	} catch(std::exception& e) {
		ROS_ERROR_STREAM_THROTTLE(2,"Exception while sending UDP message:"<<e.what()<< " Discarding message!");
	}
	if(buffer!=NULL) delete[] buffer;
}
void onRosMotionBurst1028144660(msl_actuator_msgs::MotionBurst& message) {
uint8_t* buffer = NULL;
	try{
		uint32_t serial_size = ros::serialization::serializationLength(message);
		buffer = new uint8_t[serial_size+sizeof(uint32_t)];
		ros::serialization::OStream stream(buffer+sizeof(uint32_t), serial_size);
		*((uint32_t*)buffer) = 1028144660u;
		ros::serialization::serialize(stream, message);
		// write message to UDP
		insocket->send_to(boost::asio::buffer((void*)buffer,serial_size+sizeof(uint32_t)),destEndPoint);
	} catch(std::exception& e) {
		ROS_ERROR_STREAM_THROTTLE(2,"Exception while sending UDP message:"<<e.what()<< " Discarding message!");
	}
	if(buffer!=NULL) delete[] buffer;
}
void onRosBool2802967882(std_msgs::Bool& message) {
uint8_t* buffer = NULL;
	try{
		uint32_t serial_size = ros::serialization::serializationLength(message);
		buffer = new uint8_t[serial_size+sizeof(uint32_t)];
		ros::serialization::OStream stream(buffer+sizeof(uint32_t), serial_size);
		*((uint32_t*)buffer) = 2802967882u;
		ros::serialization::serialize(stream, message);
		// write message to UDP
		insocket->send_to(boost::asio::buffer((void*)buffer,serial_size+sizeof(uint32_t)),destEndPoint);
	} catch(std::exception& e) {
		ROS_ERROR_STREAM_THROTTLE(2,"Exception while sending UDP message:"<<e.what()<< " Discarding message!");
	}
	if(buffer!=NULL) delete[] buffer;
}
void onRosRawOdometryInfo3134514216(msl_actuator_msgs::RawOdometryInfo& message) {
uint8_t* buffer = NULL;
	try{
		uint32_t serial_size = ros::serialization::serializationLength(message);
		buffer = new uint8_t[serial_size+sizeof(uint32_t)];
		ros::serialization::OStream stream(buffer+sizeof(uint32_t), serial_size);
		*((uint32_t*)buffer) = 3134514216u;
		ros::serialization::serialize(stream, message);
		// write message to UDP
		insocket->send_to(boost::asio::buffer((void*)buffer,serial_size+sizeof(uint32_t)),destEndPoint);
	} catch(std::exception& e) {
		ROS_ERROR_STREAM_THROTTLE(2,"Exception while sending UDP message:"<<e.what()<< " Discarding message!");
	}
	if(buffer!=NULL) delete[] buffer;
}
void onRosCanMsg1267609526(msl_actuator_msgs::CanMsg& message) {
uint8_t* buffer = NULL;
	try{
		uint32_t serial_size = ros::serialization::serializationLength(message);
		buffer = new uint8_t[serial_size+sizeof(uint32_t)];
		ros::serialization::OStream stream(buffer+sizeof(uint32_t), serial_size);
		*((uint32_t*)buffer) = 1267609526u;
		ros::serialization::serialize(stream, message);
		// write message to UDP
		insocket->send_to(boost::asio::buffer((void*)buffer,serial_size+sizeof(uint32_t)),destEndPoint);
	} catch(std::exception& e) {
		ROS_ERROR_STREAM_THROTTLE(2,"Exception while sending UDP message:"<<e.what()<< " Discarding message!");
	}
	if(buffer!=NULL) delete[] buffer;
}
void onRosCanMsg217678336(msl_actuator_msgs::CanMsg& message) {
uint8_t* buffer = NULL;
	try{
		uint32_t serial_size = ros::serialization::serializationLength(message);
		buffer = new uint8_t[serial_size+sizeof(uint32_t)];
		ros::serialization::OStream stream(buffer+sizeof(uint32_t), serial_size);
		*((uint32_t*)buffer) = 217678336u;
		ros::serialization::serialize(stream, message);
		// write message to UDP
		insocket->send_to(boost::asio::buffer((void*)buffer,serial_size+sizeof(uint32_t)),destEndPoint);
	} catch(std::exception& e) {
		ROS_ERROR_STREAM_THROTTLE(2,"Exception while sending UDP message:"<<e.what()<< " Discarding message!");
	}
	if(buffer!=NULL) delete[] buffer;
}
void onRosCanMsg418700403(msl_actuator_msgs::CanMsg& message) {
uint8_t* buffer = NULL;
	try{
		uint32_t serial_size = ros::serialization::serializationLength(message);
		buffer = new uint8_t[serial_size+sizeof(uint32_t)];
		ros::serialization::OStream stream(buffer+sizeof(uint32_t), serial_size);
		*((uint32_t*)buffer) = 418700403u;
		ros::serialization::serialize(stream, message);
		// write message to UDP
		insocket->send_to(boost::asio::buffer((void*)buffer,serial_size+sizeof(uint32_t)),destEndPoint);
	} catch(std::exception& e) {
		ROS_ERROR_STREAM_THROTTLE(2,"Exception while sending UDP message:"<<e.what()<< " Discarding message!");
	}
	if(buffer!=NULL) delete[] buffer;
}
void onRosCanMsg3391245383(msl_actuator_msgs::CanMsg& message) {
uint8_t* buffer = NULL;
	try{
		uint32_t serial_size = ros::serialization::serializationLength(message);
		buffer = new uint8_t[serial_size+sizeof(uint32_t)];
		ros::serialization::OStream stream(buffer+sizeof(uint32_t), serial_size);
		*((uint32_t*)buffer) = 3391245383u;
		ros::serialization::serialize(stream, message);
		// write message to UDP
		insocket->send_to(boost::asio::buffer((void*)buffer,serial_size+sizeof(uint32_t)),destEndPoint);
	} catch(std::exception& e) {
		ROS_ERROR_STREAM_THROTTLE(2,"Exception while sending UDP message:"<<e.what()<< " Discarding message!");
	}
	if(buffer!=NULL) delete[] buffer;
}
void onRosIMUData3455796956(msl_actuator_msgs::IMUData& message) {
uint8_t* buffer = NULL;
	try{
		uint32_t serial_size = ros::serialization::serializationLength(message);
		buffer = new uint8_t[serial_size+sizeof(uint32_t)];
		ros::serialization::OStream stream(buffer+sizeof(uint32_t), serial_size);
		*((uint32_t*)buffer) = 3455796956u;
		ros::serialization::serialize(stream, message);
		// write message to UDP
		insocket->send_to(boost::asio::buffer((void*)buffer,serial_size+sizeof(uint32_t)),destEndPoint);
	} catch(std::exception& e) {
		ROS_ERROR_STREAM_THROTTLE(2,"Exception while sending UDP message:"<<e.what()<< " Discarding message!");
	}
	if(buffer!=NULL) delete[] buffer;
}

boost::array<char,64000> inBuffer;
void listenForPacket() {
	insocket->async_receive_from(boost::asio::buffer(inBuffer), otherEndPoint,
        boost::bind(&handleUdpPacket, boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred));
}
void handleUdpPacket(const boost::system::error_code& error,   std::size_t bytes_transferred) {
	//std::cout << "From "<<otherEndPoint.address() << std::endl;
	if (bytes_transferred > 64000) {
		return;
	}
	if (!error) { // && otherEndPoint.address() != localIP) {
		__uint32_t id = *((__uint32_t*)(inBuffer.data()));
		//std::cout << "Got packet"<<std::endl;
		try {
			ros::serialization::IStream stream(((uint8_t*)inBuffer.data())+sizeof(__uint32_t),bytes_transferred-sizeof(__uint32_t));
			switch(id) {
				case 1334345447ul: {
				msl_actuator_msgs::BallHandleCmd m1334345447;
				ros::serialization::Serializer<msl_actuator_msgs::BallHandleCmd>::read(stream, m1334345447);

				handleBallHandleControl(m1334345447);
				break; }
				case 297244167ul: {
				msl_actuator_msgs::BallHandleMode m297244167;
				ros::serialization::Serializer<msl_actuator_msgs::BallHandleMode>::read(stream, m297244167);

				handleBallHandleMode(m297244167);
				break; }
				case 1418208429ul: {
				msl_actuator_msgs::ShovelSelectCmd m1418208429;
				ros::serialization::Serializer<msl_actuator_msgs::ShovelSelectCmd>::read(stream, m1418208429);

				handleShovelSelectControl(m1418208429);
				break; }
				case 2056271736ul: {
				msl_actuator_msgs::MotionLight m2056271736;
				ros::serialization::Serializer<msl_actuator_msgs::MotionLight>::read(stream, m2056271736);

				handleMotionLight(m2056271736);
				break; }
				case 554624761ul: {
//				process_manager::ProcessCommand m554624761;
//				ros::serialization::Serializer<process_manager::ProcessCommand>::read(stream, m554624761);
//				pub554624761.publish<process_manager::ProcessCommand>(m554624761);
				break; }
				case 2772566283ul: {
//				msl_actuator_msgs::VisionRelocTrigger m2772566283;
//				ros::serialization::Serializer<msl_actuator_msgs::VisionRelocTrigger>::read(stream, m2772566283);
//				pub2772566283.publish<msl_actuator_msgs::VisionRelocTrigger>(m2772566283);
				break; }
				case 1028144660ul: {
//				msl_actuator_msgs::MotionBurst m1028144660;
//				ros::serialization::Serializer<msl_actuator_msgs::MotionBurst>::read(stream, m1028144660);
//				pub1028144660.publish<msl_actuator_msgs::MotionBurst>(m1028144660);
				break; }
				case 2802967882ul: {
//				std_msgs::Bool m2802967882;
//				ros::serialization::Serializer<std_msgs::Bool>::read(stream, m2802967882);
//				pub2802967882.publish<std_msgs::Bool>(m2802967882);
				break; }
				case 3134514216ul: {
				msl_actuator_msgs::RawOdometryInfo m3134514216;
				ros::serialization::Serializer<msl_actuator_msgs::RawOdometryInfo>::read(stream, m3134514216);
				handleRawOdometryInfo(m3134514216);
				break; }
				case 1267609526ul: {
				// CanSub
				msl_actuator_msgs::CanMsg m1267609526;
				ros::serialization::Serializer<msl_actuator_msgs::CanMsg>::read(stream, m1267609526);
				handleCanSub(m1267609526);

				break; }
				case 217678336ul: {
//				msl_actuator_msgs::CanMsg m217678336;
//				ros::serialization::Serializer<msl_actuator_msgs::CanMsg>::read(stream, m217678336);
//				pub217678336.publish<msl_actuator_msgs::CanMsg>(m217678336);
				break; }
				case 418700403ul: {
//				msl_actuator_msgs::CanMsg m418700403;
//				ros::serialization::Serializer<msl_actuator_msgs::CanMsg>::read(stream, m418700403);
//				pub418700403.publish<msl_actuator_msgs::CanMsg>(m418700403);
				break; }
				case 3391245383ul: {
//				msl_actuator_msgs::CanMsg m3391245383;
//				ros::serialization::Serializer<msl_actuator_msgs::CanMsg>::read(stream, m3391245383);
//				pub3391245383.publish<msl_actuator_msgs::CanMsg>(m3391245383);
				break; }
				case 3455796956ul: {
//				msl_actuator_msgs::IMUData m3455796956;
//				ros::serialization::Serializer<msl_actuator_msgs::IMUData>::read(stream, m3455796956);
//				pub3455796956.publish<msl_actuator_msgs::IMUData>(m3455796956);
				break; }

				default:
					std::cerr << "Cannot find Matching topic:" << id << std::endl;
			}
		}
		catch(std::exception& e) {
			ROS_ERROR_STREAM_THROTTLE(2,"Exception while receiving DDS message:"<<e.what()<< " Discarding message!");
		}

	}
	listenForPacket();
	return;
}
void run_udp() {
	io_service.run();
}










void controlBallHandle() {
	const msl_actuator_msgs::BallHandleMode msg;
	unique_lock<mutex> l_bhl(threw[0].mtx);
	while(th_activ) {
		threw[0].cv.wait(l_bhl, [&] { return !th_activ || threw[0].notify; }); // protection against spurious wake-ups
		if (!th_activ)
			return;

		try {
			if (ballHandle.getMode() == msg.REMOTE_CONTROL) {
				ballHandle.checkTimeout();
			}
			if (ballHandle.getMode() == msg.AUTONOMOUS_CONTROL) {
				ballHandle.dribbleControl();
			}
		} catch (exception &e) {
			cout << "BallHanlde left: " << e.what() << endl;
		}

		threw[0].notify = false;
	}
}

void contolShovelSelect() {
	unique_lock<mutex> l_shovel(threw[1].mtx);
	while(th_activ) {
		threw[1].cv.wait(l_shovel, [&] { return !th_activ || threw[1].notify; }); // protection against spurious wake-ups
		if (!th_activ)
			return;

		try {
			shovel.checkTimeout(time_now);
		} catch (exception &e) {
			cout << "Shovel: " << e.what() << endl;
		}

		threw[1].notify = false;
	}
}

void getLightbarrier() {
	std_msgs::Bool msg;
	unique_lock<mutex> l_light(threw[2].mtx);
	while(th_activ) {
		threw[2].cv.wait(l_light, [&] { return !th_activ || threw[2].notify; }); // protection against spurious wake-ups
		if (!th_activ)
			return;

		try {
			msg.data = lightbarrier.checkLightBarrier();
			onRosBool2802967882(msg);
			//lbiPub->publish(msg);
		} catch (exception &e) {
			cout << "ADC: " << e.what() << endl;
		}

		threw[2].notify = false;
	}
}

void getSwitches() {
	supplementary::SystemConfig* sc;
	sc = supplementary::SystemConfig::getInstance();
	enum	Pin { sw_vision, sw_bundle, sw_power, led_power, led_bundle, led_vision };
	int		ownID = (*sc)["bbb"]->get<int>("BBB.robotID",NULL);
	msl_actuator_msgs::VisionRelocTrigger msg_v;
	process_manager::ProcessCommand msg_pm;

	const char *pin_names[] = { "P9_11", "P9_13", "P9_15", "P9_23", "P9_41", "P9_42" }; /* sw_vis, sw_bun, sw_pwr, led_pwr, led_bun, led_vis */
	BeagleGPIO *gpio = BeagleGPIO::getInstance();
	BeaglePins *pins = gpio->claim((char**) pin_names, 6);

	int outputIdxs[] = { led_power, led_bundle, led_vision };
	pins->enableOutput(outputIdxs, 3);

	unique_lock<mutex> l_switches(threw[3].mtx);
	while(th_activ) {
		threw[3].cv.wait(l_switches, [&] { return !th_activ || threw[3].notify; }); // protection against spurious wake-ups
		if (!th_activ)
			return;

		static bool		state[3] = {false, false, false};
		bool newstate[3];
		int	sw[3] = {1, 1, 1};

		try {
			// TODO überprüfen, ob Auslesen mit der API funktioniert
			sw[sw_vision] = pins->getBit(sw_vision);
			sw[sw_bundle]	= pins->getBit(sw_bundle);
			sw[sw_power]	= pins->getBit(sw_power);
		} catch (exception &e) {
			cout << "Buttons: " << e.what() << endl;
		}

		for (int i = 0; i <= 2; i++) {
			if(sw[i] == 1)
				newstate[i] = false;
			else if (sw[i] == 0)
				newstate[i] = true;
			else
				cout << "Button " << i << " failure" << endl;
		}

		if (newstate[sw_bundle] != state[sw_bundle]) {
			state[sw_bundle] = newstate[sw_bundle];

			if (state[sw_bundle]) {
				static uint8_t bundle_state = 0;

				msg_pm.receiverId = ownID;
				msg_pm.robotIds = {ownID};
				msg_pm.processKeys = {2,3,4,5,7};
				msg_pm.paramSets = {1,0,0,0,3};

				if (bundle_state == 0) {		// Prozesse starten
					bundle_state = 1;
					msg_pm.cmd = 0;
					pins->setBit(led_bundle);	// LED an
				} else if (bundle_state == 1) {	// Prozesse stoppen
					bundle_state = 0;
					msg_pm.cmd = 1;
					pins->clearBit(led_bundle);	// LED aus
				}
				onRosProcessCommand554624761(msg_pm);
				//brtPub->publish(msg_pm);
			}
		}

		if (newstate[sw_vision] != state[sw_vision]) {
			state[sw_vision] = newstate[sw_vision];

			if (state[sw_vision]) {
				msg_v.receiverID = ownID;
				msg_v.usePose = false;
				onRosVisionRelocTrigger2772566283(msg_v);
				//vrtPub->publish(msg_v);
				pins->setBit(led_vision);	// Vision-LED an
			} else {
				pins->clearBit(led_vision);	// Vision-LED aus
			}
		}

		if (newstate[sw_power] != state[sw_power]) {
			state[sw_power] = newstate[sw_power];

			if (state[sw_power]) {
				std_msgs::Empty msg;
				//TODO not sent yet -> copy from generated code!
				//flPub->publish(msg);
				pins->setBit(led_power);	// Power-LED an
			} else {
				pins->clearBit(led_power);	// Power-LED aus
			}
		}

		threw[3].notify = false;
	}
	delete gpio;
}

void getIMU() {
	unique_lock<mutex> l_imu(threw[4].mtx);
	while(th_activ) {
		threw[4].cv.wait(l_imu, [&] { return !th_activ || threw[4].notify; }); // protection against spurious wake-ups
		if (!th_activ)
			return;

		msl_actuator_msgs::IMUData msg;
		try {
			lsm9ds0.getData(time_now);
			msg = lsm9ds0.sendData(time_now);
			onRosIMUData3455796956(msg);
		} catch (exception &e) {
			cout << "IMU: " << e.what() << endl;
		}

		threw[4].notify = false;
	}
}

void getOptical() {
	unique_lock<mutex> l_optical(threw[5].mtx);
	while(th_activ) {
		threw[5].cv.wait(l_optical, [&] { return !th_activ || threw[5].notify; }); // protection against spurious wake-ups
		if (!th_activ)
			return;

		msl_actuator_msgs::MotionBurst msg;
		try {
			adns3080.update_motion_burst();
			msg = adns3080.getMotionBurstMsg();
			onRosMotionBurst1028144660(msg);
		} catch (exception &e) {
			cout << "Optical Flow: " << e.what() << endl;
		}

		threw[5].notify = false;
	}
}

void exit_program(int sig) {
	ex = true;
	th_activ = false;
	for (int i=0; i<7; i++)
		threw[i].cv.notify_all();
}




int main(int argc, char** argv) {
	ros::Time::init();
	ros::Rate loop_rate(30);		// in Hz

	supplementary::SystemConfig* sc;
	sc = supplementary::SystemConfig::getInstance();

	thread th_controlBallHandle(controlBallHandle);
	thread th_controlShovel(contolShovelSelect);
	thread th_lightbarrier(getLightbarrier);
	thread th_switches(getSwitches);
	thread th_imu(getIMU);

	// I2C
	bool i2c = myI2C.open(ReadWrite);
	bool spi = mySpi.open(ReadWrite);
	bool imu = lsm9ds0.init();

	supplementary::Configuration *proxyconf = (*sc)["msl_bbb_proxy"];
	std::string baddress = proxyconf->get<std::string>("UdpProxy","MulticastAddress",NULL);
	unsigned short port = (unsigned short)proxyconf->get<int>("UdpProxy","Port",NULL);
	multiCastAddress = boost::asio::ip::address::from_string(baddress);
	destEndPoint = udp::endpoint(multiCastAddress,port);
	std::cout<<"Opening to "<<multiCastAddress <<std::endl;
	insocket = new udp::socket(io_service,udp::endpoint(multiCastAddress,port));
	insocket->set_option(boost::asio::ip::multicast::enable_loopback(false));
	insocket->set_option(boost::asio::ip::multicast::join_group(multiCastAddress));
	listenForPacket();

	usleep(50000);

	// CAN hack
	canHandler.Start();

	boost::thread iothread(run_udp);
	std::cout << "Udp connection active..." <<std::endl;

	(void) signal(SIGINT, exit_program);
	while(!ex) {
		gettimeofday(&time_now, NULL);

		// Thread Notify
		for (int i=0; i<6; i++) { // TODO remove magic number
			if (threw[i].notify) {
				cerr << "Thread " << i << " requires to much time, iteration is skipped" << endl;
			} else {
				threw[i].notify = true;
			}
			threw[i].cv.notify_all();
		}

		loop_rate.sleep();
	}
    io_service.stop();
    iothread.join();
    canHandler.Stop();

	return 0;
}
