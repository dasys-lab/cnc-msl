

#include <stdio.h>
#include <iostream>
#include <string>
#include <signal.h>
#include "usbcanconnection.h"
#include "Can.h"


#define BUFSIZE 64



class UsbCanConsole : public CanListener {
	public:
		UsbCanConsole(int showcmp) {
			us = new UsbCanConnection("can0");
			us->SetReceiver(this);
			displCmp = showcmp;		


		}
		void Close() {
			us->Stop();			
		}
		void Start() {
			//int ret=0;
			//unsigned int canid;
			us->Start();
			std::string input;
			unsigned int canid = 0;
			canid |= ReKick;
			canid |= (Eth2Can << 8) & 0xFF00;
			canid |= (CanPriNorm<<16) & 0xFF0000;
			while(1) { 
					getline(std::cin,input);
					if (input.compare("quit")==0) break;
					
					if (input.compare("rekick")==0) {
						canid = 0;
						canid |= ReKick;
						canid |= (Eth2Can << 8) & 0xFF00;
						canid |= (CanPriNorm<<16) & 0xFF0000;
						printf("Use Rekick\n");
						continue;
					}
					
					if (input.compare("actuator")==0) {
						canid = 0;
						canid |= BallHandler;
						canid |= (Eth2Can << 8) & 0xFF00;
						canid |= (CanPriNorm<<16) & 0xFF0000;
						printf("Use Actuator\n");
						continue;
					}

					if (input.compare("compass")==0) {
						canid = 0;
						canid |= Compass;
						canid |= (Eth2Can << 8) & 0xFF00;
						canid |= (CanPriNorm<<16) & 0xFF0000;
						printf("Use Compass\n");
						continue;
					}


					int len = input.length();
					
					if (len > 8) len = 8;
					us->SendExCanMsg(canid,(unsigned char*)input.c_str(),len);
			}
			
			exit(1);
		}
		void Receive(unsigned int canid,unsigned char* data, int len) {
			if (((canid & 0xFF00)>>8) == Compass) {
				if (len<2) {
					printf("Unsuspected short can package from compass!\n");
				} else if (displCmp>0) {
					int degree = (int)data[0]<<8;
					degree+= (int)data[1];
					printf("%d\n",degree);
				}
				
			}
			else {
				for (int x=0; x < len; x++) {
					std::cout << (char)data[x];
				}
				std::cout << std::endl;
			}
		}
	protected:
		UsbCanConnection *us;
		int displCmp;
};

UsbCanConsole *ucc;

void int_handler(int sig) {
	if (ucc) {
		ucc->Close();
	}
	exit(2);
}
int main(int argc, char** argv) {
	signal(SIGINT,int_handler);
	int displcmp = 1;	
	if (argc>1 && strcmp(argv[1],"-nocmp")==0) {
			displcmp = 0;
	}
	ucc = new UsbCanConsole(displcmp);
	ucc->Start();
}


