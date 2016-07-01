/*
 * CanReceiver.cpp
 *
 *  Created on: 30.06.2016
 *      Author: paspartout
 */

#include "../include/CanHandler.h"

CanReceiver::CanReceiver() {
	// TODO Auto-generated constructor stub

}

CanReceiver::~CanReceiver() {
	// TODO Auto-generated destructor stub
}



void Receive()
{
	int found = 0;
	unsigned int id = ((canid & 0xFF00)>>8);
	for(unsigned int i=0; i<receivers.size(); i++) {
		//printf("receiver %u\n",receivers[i]);
		//printf("id is : %u \n",id);
		if( id == receivers[i])
		{
			found=1;

			CanMsg cm;
// 				cm.header = 0x0;
// 				cm.priority = (canid>>16) & 0xFF; //Priority
// 				cm.sender = (canid>>8) & 0xFF; //Sender
// 				cm.receiver = (canid)  & 0xFF;  //receiver
// 				cm.length = (len<<1);
			cm.id = id;
			for(int i=0; i<len; i++)
			{
				cm.data.push_back(data[i]);
			}

			if( id == Compass )
			{
				compass.publish(cm);
				//printf("get compass val from canbus!\n");
			}
			else if( id == ReKick )
			{
				rekick.publish(cm);
				/*printf("get rekick val from canbus!\n");
				for(unsigned int i=0; i<cm.data.size(); i++)
				{
					printf("%u\n",cm.data[i]);
				}*/

			}
			else if( id == BallHandler )
			{
				ballhandler.publish(cm);
				//printf("get actuator val from canbus!\n");

			}
			else
			{
				fprintf(stderr,"Unkown canid received: 0x%x\n",canid);
			}
			break;
		}
	}
	if(found <= 0) {
		fprintf(stderr,"Unkown canid received: 0x%x\n",canid);
	}
}

