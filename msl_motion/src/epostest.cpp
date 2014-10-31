//#include "driver/eposusb.h"
#include "driver/eposcan.h"
#include <udpcanconnection.h>
#include <usbcanconnection.h>
#include "settings.h"
#include <stdio.h>

//Controlling::EposUSB *ep;
Controlling::EposCan *ep;
UdpCanConnection *uc;
UsbCanConnection *us;


int main(int argc, char** argv) {
	//char ipstr[12] = "192.168.0.2";
	printf("Here\n");
	settings_init(); 
	/*ep = new Controlling::EposUSB::EposUSB(1);
	
	while(ep->InitConnection()<0) {
		printf("Cannot establish usb connection!\n");
		usleep(500000);
	}

	//unsigned char data[4] = {0x81,0x20,0x00,0x01};
	//ep->SendCommand(0x10,data,4);
	//ep->QueryObject(1,data);
	
	while(1)  {
	ep->SendLifeGuard();	
	//unsigned char data[4] = {0x00,0x00,0x00,0x01};
	//ep->SendCanFrame(0x600,2,data, 2);	

	ep->Listen();
	ep->Listen();
	ep->Listen();
	ep->Listen();
	sleep(1);	
	}*/
	us = new UsbCanConnection("can0");
	//ep = new Controlling::EposCan::EposCan(4,us);

	us->Start();
	usleep(100000);
	//unsigned char bla[7] = "bla123";
	while(1) {
		us->SendExCanRtr((unsigned short)0x1123);
		usleep(100000);
	}
//	exit(1);

//	uc = new UdpCanConnection::UdpCanConnection(ipstr,10003);
//	ep = new Controlling::EposCan::EposCan(4,uc);

//	uc->Start();
//	usleep(30000);

	while(ep->InitAllNodes()<=0) {
        std::cout << "Could not initialise all controllers! Retrying...\n";
		usleep(500000);
	};
	/*ep->ResetAllNodes();	
	ep->SendLifeGuard();
	
	ep->SendLifeGuard();
	ep->ResetFaults();
	*/
	//usleep(60000);
	//ep->ResetCommunication();

	while(1) {
		ep->SendLifeGuard();
		ep->ReadErrorRegister();
		usleep(500000);
	}
	//ep->DisableAllNodes();
	/*while(ep->InitAllNodes()<=0) {
		usleep(500000);
	};*/
	while(1) {
		ep->SendLifeGuard();
		ep->Trigger(1);
		usleep(300000);

	}

}
