/*
 * CanReceiver.h
 *
 *  Created on: 30.06.2016
 *      Author: paspartout
 */

#include "ros/ros.h"
#include "msl_actuator_msgs/CanMsg.h"
#include "rosmsgfuncs.h"

#include <stdio.h>
#include <signal.h>
#include <sys/time.h>
#include <vector>

#include <Can.h>
#include <usbcanconnection.h>

using namespace std;
using msl_actuator_msgs::CanMsg;

#ifndef CNC_MSL_MSL_BEAGLE_BOARD_BLACK_UDP_SRC_CANRECEIVER_H_
#define CNC_MSL_MSL_BEAGLE_BOARD_BLACK_UDP_SRC_CANRECEIVER_H_

class CanHandler: public CanListener {

public:

	CanHandler() {
		us = new UsbCanConnection("can0");
		us->SetReceiver(this);

		//add id's for can devices
		receivers.push_back(Compass);
		receivers.push_back(ReKick);
		receivers.push_back(BallHandler);
	}

	void Close() {
		us->Stop();
	}

	void sendCanMsg(const msl_actuator_msgs::CanMsg &msg) {
		printf("write data to can!\n");

		unsigned char* p = (unsigned char*) msg.data.data();
		us->SendExCanMsg(msg.id, p, msg.data.size());
	}

	void Start() {

		us->Start();
		std::cerr << "Staring CanReceiver" << std::endl;
	}

	void Stop() {
		us->Stop();
		std::cerr << "Stopping CanReceiver" << std::endl;
	}

protected:
	//while reading from usb -> publisher
	//ros::AsyncSpinner *spinner;
	//ros::Time lastMotion;

	// publisher for direct bundle restart trigger

	UsbCanConnection *us;
	vector<unsigned short> receivers;

	void Receive(unsigned int canid, unsigned char* data, int len) {
		bool found = false;
		unsigned int id = ((canid & 0xFF00) >> 8);
		for (unsigned int i = 0; i < receivers.size(); i++) {
			//printf("receiver %u\n",receivers[i]);
			//printf("id is : %u \n",id);
			if (id == receivers[i]) {
				found = true;

				CanMsg cm;
				cm.id = id;

				for (int i = 0; i < len; i++) {
					cm.data.push_back(data[i]);
				}

				switch (id) {
				case Compass:
					break;
				case ReKick:
					printf("sending reckick can msg\n");
					onRosCanMsg418700403(cm);
					break;
				case BallHandler:
					break;
				default:
					break;
				}
			}
		}

		if (!found) {
			fprintf(stderr, "Unkown canid received: 0x%x\n", canid);
		}

	}

	void resetInterface() {
		Close();
		int e = system("sudo ifdown can0");
		sleep(1);
		us = new UsbCanConnection("can0");
		us->SetReceiver(this);
		e = system("sudo ifup can0");
		sleep(1);
		if (!e)
			printf("error reseting!\n");

		//lastMotion = ros::Time::now();
		Start();
	}
};

#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_UDP_SRC_CANRECEIVER_H_ */
