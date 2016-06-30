/*
 * CanReceiver.h
 *
 *  Created on: 30.06.2016
 *      Author: paspartout
 */

#include "ros/ros.h"
#include "msl_actuator_msgs/CanMsg.h"

#include <stdio.h>
#include <signal.h>
#include <sys/time.h>
#include <vector>

#include <Can.h>
#include <usbcanconnection.h>

using namespace std;

#ifndef CNC_MSL_MSL_BEAGLE_BOARD_BLACK_UDP_SRC_CANRECEIVER_H_
#define CNC_MSL_MSL_BEAGLE_BOARD_BLACK_UDP_SRC_CANRECEIVER_H_

class CanHandler : public CanListener
{

public:

	CanHandler()
	{
		us = new UsbCanConnection("can0");
		us->SetReceiver(this);

		//add id's for can devices
		receivers.push_back(Compass);
		receivers.push_back(ReKick);
		receivers.push_back(BallHandler);
	}

	void Close()
	{
		us->Stop();
	}

	void sendCanMsg(const msl_actuator_msgs::CanMsg &msg)
	{
		printf("write data to can!\n");

		unsigned char* p = (unsigned char*)msg.data.data();
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

	void Receive(unsigned int canid,unsigned char* data, int len)
	{
//		int found = 0;
//		unsigned int id = ((canid & 0xFF00)>>8);
//		for(unsigned int i=0; i<receivers.size(); i++) {
//			//printf("receiver %u\n",receivers[i]);
//			//printf("id is : %u \n",id);
//			if( id == receivers[i])
//			{
//				found=1;
//
//				CanMsg cm;
//// 				cm.header = 0x0;
//// 				cm.priority = (canid>>16) & 0xFF; //Priority
//// 				cm.sender = (canid>>8) & 0xFF; //Sender
//// 				cm.receiver = (canid)  & 0xFF;  //receiver
//// 				cm.length = (len<<1);
//				cm.id = id;
//				for(int i=0; i<len; i++)
//				{
//					cm.data.push_back(data[i]);
//				}
//
//				if( id == Compass )
//				{
//					compass.publish(cm);
//					//printf("get compass val from canbus!\n");
//				}
//				else if( id == ReKick )
//				{
//					rekick.publish(cm);
//					/*printf("get rekick val from canbus!\n");
//					for(unsigned int i=0; i<cm.data.size(); i++)
//					{
//						printf("%u\n",cm.data[i]);
//					}*/
//
//				}
//				else if( id == BallHandler )
//				{
//					ballhandler.publish(cm);
//					//printf("get actuator val from canbus!\n");
//
//				}
//				else
//				{
//					fprintf(stderr,"Unkown canid received: 0x%x\n",canid);
//				}
//				break;
//			}
//		}
//		if(found <= 0) {
//			fprintf(stderr,"Unkown canid received: 0x%x\n",canid);
//		}
	}

	void resetInterface()
	{
		Close();
		int e = system("sudo ifdown can0");
		sleep(1);
		us = new UsbCanConnection("can0");
		us->SetReceiver(this);
		e = system("sudo ifup can0");
		sleep(1);
		if(!e) printf("error reseting!\n");

		//lastMotion = ros::Time::now();
		Start();
	}
};



#endif /* CNC_MSL_MSL_BEAGLE_BOARD_BLACK_UDP_SRC_CANRECEIVER_H_ */
