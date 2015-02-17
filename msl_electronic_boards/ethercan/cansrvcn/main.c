#include "main.h"
#include <stdio.h>
#include <getopt.h>

#include <fcntl.h>


#include "udp.h"
#include "can.h"
#include "utils.h"


int sendCnt       = 0; /* sended messages counter  */
int recvCnt       = 0; /* received messages counter  */
int recvCpcCnt    = 0; /* received messages (CAN) counter  */
int sendUdpCnt    = 0; /* received frames (UDP) counter  */
int sendCpcCnt	  = 0;
int recvUdpCnt	  = 0;

char devString[64] = "/dev/can0";
char ipString[32] = "192.168.0.5";

int nWDogFD = 0;
unsigned char isBusOff=0;
//extern char udp_in_buffer[UDP_RECEIVE_BUFFER_SIZE];

int main(int argc, char **argv)
{

	int can_handle, socket;
	int handleBusOff;
	int port=10003, c;
	int baudrate=1000;
	CPC_MSG_T params;

/* START OF GETOPT-NEEDED VARIABLES */
	extern char * optarg;
	char        * options;
	char        * pname;
	
	static struct option long_options[] = {
		// option, has arg, flag, val
		{"trigger-watchdog", 0, 0, 'w'},
		{"port", 1, 0, 'p'},
		{"ip-address", 1, 0, 'i'},
		{"device-file", 1, 0, 'd'},
		{"baudrate", 1, 0, 'b'},
		{0, 0, 0, 0}
	};
	int option_index = 0;
	pname   = *argv;       /* pointer to commandlineparameter  */
	options = "?hp:i:d:b:w";  /* parameterlist for getopt()  */



	/* END OF GETOPT-NEEDED VARIABLES */

	printf("Starting..\n");

	/* START OF WHILE-LOOP FOR GETOPT */
	while ((c = getopt_long(argc, argv, options, long_options, &option_index)) != EOF) {
		switch(c) {
			case 'b':
				baudrate = atoi(optarg);
				if (baudrate > 1000 || baudrate < 10) {
					printf("Baudrate not supported: %d\n",baudrate);
					exit(1);
				}
				break;				
			case -1: /* error */
			case '?': /* help */
			case 'h': /* help */
				printf("Usage: cansrvcn [-p port] [-i destination_ip] [-w] [-d can_device] [-b baudrate]\n");
				exit(1);
			case 'p': /* the port were the server listens */
				port = atoi(optarg);
				if(port <= 0) {
					printf("ERROR: No valid port\n");
					exit(1);
				}
			break;
			case 'i':
				strncpy(ipString, optarg, sizeof(ipString));				
				break;
			case 'd':
				strncpy(devString, optarg, sizeof(devString));
				break;
			case 'w':
				nWDogFD = open("/dev/watchdog", O_WRONLY);
				if(nWDogFD < 0){
					perror("WARNING: Could not open watchdog");
				}
				break;
		}
	}
/* END OF WHILE-LOOP FOR GETOPT */

	printf("Opening Socket at port %d for IP %s\n",port,ipString);

	socket = setupUdpSockets(port, ipString);
	if (socket <= 0) {
		perror("Could not open socket!");
		exit(1);
	}

	printf("Opening Can device: %s\n",devString);
	
	can_handle = initCan(devString);
	if (can_handle <= 0 ) {
		perror("Could not open can device!");
		exit(1);
	}

	// set baudrate
	memset(&params, 0, sizeof(CPC_MSG_T));
	params.type = CPC_CMD_T_CAN_PRMS;
	params.length = sizeof(CPC_SJA1000_PARAMS_T)+1; // 1byte is for cc_type
	params.msg.canparams.cc_type = SJA1000;
	params.msg.canparams.cc_params.sja1000.mode = 0x00;
	params.msg.canparams.cc_params.sja1000.acc_code0 = 0x00;
	params.msg.canparams.cc_params.sja1000.acc_code1 = 0x00;
	params.msg.canparams.cc_params.sja1000.acc_code2 = 0x00;
	params.msg.canparams.cc_params.sja1000.acc_code3 = 0x00;
	params.msg.canparams.cc_params.sja1000.acc_mask0 = 0xFF;
	params.msg.canparams.cc_params.sja1000.acc_mask1 = 0xFF;
	params.msg.canparams.cc_params.sja1000.acc_mask2 = 0xFF;
	params.msg.canparams.cc_params.sja1000.acc_mask3 = 0xFF;
	params.msg.canparams.cc_params.sja1000.outp_contr = 0xda;
	
	switch(baudrate){
		case 1000:
			params.msg.canparams.cc_params.sja1000.btr0=0x00;
			params.msg.canparams.cc_params.sja1000.btr1=0x14;
			break;
		case 800:
			params.msg.canparams.cc_params.sja1000.btr0=0x00;
			params.msg.canparams.cc_params.sja1000.btr1=0x16;
			break;
		case 500:
			params.msg.canparams.cc_params.sja1000.btr0=0x00;
			params.msg.canparams.cc_params.sja1000.btr1=0x1c;
			break;
		case 250:
			params.msg.canparams.cc_params.sja1000.btr0=0x01;
			params.msg.canparams.cc_params.sja1000.btr1=0x1c;
			break;
		case 125:
			params.msg.canparams.cc_params.sja1000.btr0=0x03;
			params.msg.canparams.cc_params.sja1000.btr1=0x1c;
			break;
		case 100:
			params.msg.canparams.cc_params.sja1000.btr0=0x04;
			params.msg.canparams.cc_params.sja1000.btr1=0x1c;
			break;
		case 50:
			params.msg.canparams.cc_params.sja1000.btr0=0x09;
			params.msg.canparams.cc_params.sja1000.btr1=0x1c;
			break;
		case 25:
			params.msg.canparams.cc_params.sja1000.btr0=0x13;
			params.msg.canparams.cc_params.sja1000.btr1=0x1c;
			break;
		case 20:
			params.msg.canparams.cc_params.sja1000.btr0=0x18;
			params.msg.canparams.cc_params.sja1000.btr1=0x1c;
			break;
		case 10:
			params.msg.canparams.cc_params.sja1000.btr0=0x31;
			params.msg.canparams.cc_params.sja1000.btr1=0x1c;
			break;
		default:
			printf("Baudrate %d is not supported\n", baudrate);
			exit(1);
	}

	write(can_handle, &params, sizeof(CPC_MSG_T));

	printf("\nCAN Parameter:\n");

	printf("mode      : %2.2Xh\n", params.msg.canparams.cc_params.sja1000.mode);
	printf("btr0      : %2.2Xh\n", params.msg.canparams.cc_params.sja1000.btr0);
	printf("btr1      : %2.2Xh\n", params.msg.canparams.cc_params.sja1000.btr1);
	printf("accCode0  : %2.2Xh\n", params.msg.canparams.cc_params.sja1000.acc_code0);
	printf("accCode1  : %2.2Xh\n", params.msg.canparams.cc_params.sja1000.acc_code1);
	printf("accCode2  : %2.2Xh\n", params.msg.canparams.cc_params.sja1000.acc_code2);
	printf("accCode3  : %2.2Xh\n", params.msg.canparams.cc_params.sja1000.acc_code3);
	printf("accMask0  : %2.2Xh\n", params.msg.canparams.cc_params.sja1000.acc_mask0);
	printf("accMask1  : %2.2Xh\n", params.msg.canparams.cc_params.sja1000.acc_mask1);
	printf("accMask2  : %2.2Xh\n", params.msg.canparams.cc_params.sja1000.acc_mask2);
	printf("accMask3  : %2.2Xh\n", params.msg.canparams.cc_params.sja1000.acc_mask3);


	init_LED("/dev/led");
	//exit(0);
	SwitchLED(LED_PROGRAM_RUNNING, LED_ON);
	SwitchLED(LED_CONN_STATE,      LED_OFF);
	SwitchLED(LED_BUS_OFF,         LED_ON);

	//nWDogFD = open("/dev/watchdog", O_WRONLY);
	while(1) {
		processReadCpc(can_handle);
		processWriteUdp();

		processReadUdp();
		processWriteCpc(can_handle);

		//sendUdp(test, 10);
		//sleep(1);
		/*count=receiveUdp();		
		printf("Received %d bytes\n",count);
		printf(udp_in_buffer);*/
		if (nWDogFD > 0) {
			triggerWatchdog();
		}

		if(isBusOff) {
			SwitchLED(LED_BUS_OFF, LED_OFF);
			printf("Bus is off!\n");
			isBusOff = 0;			
			StartTimer(&busoff_ref, &busoff_cur);
			handleBusOff = 1;
		}

		if(handleBusOff>0 && (busoff_ref.tv_sec != 0 || busoff_ref.tv_usec != 0)) {
			UpdateTimer(&busoff_cur);
			if(CheckTimer(busoff_ref, busoff_cur, 100)) {
				// reinit
				write(can_handle, &params, sizeof(CPC_MSG_T));
						
				StopTimer(&busoff_ref, &busoff_cur);
				handleBusOff = 0;
				SwitchLED(LED_BUS_OFF, LED_ON);
				printf("Bus back on\n");
			}
		}
	}

	return 0;
}
void triggerWatchdog()
{
	unsigned char magic = 'V';
	
	write(nWDogFD, &magic, 1);
	fsync(nWDogFD);
	
	return;
}
