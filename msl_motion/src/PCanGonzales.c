

#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>
//#include <time.h>
#include <libpcan.h>
#include "pcanconnect.h"
#include "MCDCProtocol.h"
#include "MCDC.h"
#include "settings.h"
#include "gonzales.h"
#include "RosHelper.h"
#include "logging.h"
#include "iostream"

/*
typedef struct
{
  DWORD ID;              // 11/29 bit code
  BYTE  MSGTYPE;         // bits of MSGTYPE_*
  BYTE  LEN;             // count of data bytes (0..8)
  BYTE  DATA[8];         // data bytes, up to 8
} TPCANMsg;              // for PCAN_WRITE_MSG

typedef struct
{
  TPCANMsg Msg;          // the above message
  DWORD    dwTime;       // a timestamp in msec, read only
  WORD     wUsec;        // remainder in micro-seconds
} TPCANRdMsg;            // for PCAN_READ_MSG
*/

extern gonzales_state gonz_state;


void processCanMsg(TPCANRdMsg* msgbuf) {
//	printf("callback!\n");
	TPCANMsg* msg = &(msgbuf->Msg);
	unsigned char nodeid = msg->ID & CAN_NODE_ID_MASK;
	switch(msg->ID & CAN_MSG_ID_MASK) {
	    case CAN_ID_GET_TRACE:
            //printf("trace\n");
            mcdc_processTraceData(nodeid,msg->DATA,msg->LEN);
            break;
	    case CAN_ID_ERROR:
            printf("emergency message from node %d!\n",nodeid);
            gonz_handleMCDCError(nodeid,(msg->DATA[1]<<8)+msg->DATA[0]);
            break;
		case CAN_ID_SDO_RESPONSE:
            //printf("sdo response\n");
            mcdc_processSDOResponse(nodeid,msg->DATA,msg->LEN);
		break;
		case CAN_ID_STATUS:
           // printf("status change!\n");
            mcdc_update_statusword(nodeid-1,(msg->DATA[1]<<8)+msg->DATA[0]);
            break;
        case CAN_ID_NMT_STATUS:
            //printf("nmt status\n");
            mcdc_processNMTResponse(nodeid,msg->DATA,1);
            break;
        case CAN_ID_PDO2_RESPONSE:
            mcdc_processPDO2Response(nodeid,msg->DATA,msg->LEN);
            break;
		default:
			printf("unknown package\n");
	}
}


int main(int argc, char** argv) {
	MotionInfoPtr cmd;
	int saveStuff = -1;

	mcdc_load(); //load mcdc specific structures
	settings_init(); //init settings struct
	gonz_init(); //init main controller
    logging_init();

	for (int i=0; i<argc; i++) {
		if (strcmp(argv[i],"--test")==0) {
			printf("Test Mode\n");
			gonz_set_mode(GONZ_MODE_TEST);
		}
		if (strcmp(argv[i],"--save")==0) {
			printf("Saveing Stuff...\n");
			saveStuff = 1;
		}
	}





	enableCanEvent(&processCanMsg); //set the can calback

	int caninited = can_init(); //init can communication

	printf("Can Init Status:%d\n",caninited);
	can_startListener(); //start can listener thread
	int initialised = 0;

	if (gonz_get_mode() == GONZ_MODE_NORMAL) {
		printf("Spica Init:\n");
	    RosHelper::initialize();
	}
    //RosHelper::sendInfo("Motion Initialiasing...");
	do {
        initialised = mcdc_init_controllers(); //init the controllers
		if (!initialised) {
                can_close();
                //RosHelper::sendWarning("Cannot Init Controllers");
                usleep(500000);
                can_init();
        }
	} while(!initialised);

	if(saveStuff > 0) {
		printf("Saving...\n");
		mcdc_save_all();
		sleep(5);
		exit(1);
	}
    //RosHelper::sendInfo("Motion Initialiased.");
	//unsigned char buffer[5] = {CAN_CMD_SET_VELOCITY,0,0,0,0};
	//INT2BYTEPOS(12*60,buffer,1);
        gonz_state.currentMotionGoal.x = 0;
        gonz_state.currentMotionGoal.y = 0;
        gonz_state.currentMotionGoal.rotation = 0;

	while(1) {
		//char charbuf[256];
	    //writeCanMsg(CAN_ID_PDO2_CMD,1,buffer,5);

        /*
        gonz_state.currentMotionGoal.x = 100;
        gonz_state.currentMotionGoal.y = 0;
        gonz_state.currentMotionGoal.rotation = 0;
        */
		switch(gonz_get_mode()) {
			case GONZ_MODE_NORMAL:
				cmd = RosHelper::getMotion();
				if (cmd) {
//				    printf("GOT COMMAND\n");
					gonz_state.currentMotionGoal.rotation=(cmd->getRotation()*1024);
					gonz_state.currentMotionGoal.x=(cmd->getTranslation()*cos(cmd->getAngle()));
					gonz_state.currentMotionGoal.y=(cmd->getTranslation()*sin(cmd->getAngle()));
				}
				printf("MM CURCMD %f\t%f\t%f\n",gonz_state.currentMotionGoal.x,gonz_state.currentMotionGoal.y,gonz_state.currentMotionGoal.rotation);
		        gonz_main();
				break;
			case GONZ_MODE_TEST:
                gonz_test_loop();
				break;
			default:
				printf("Unknown Mode, doing nothing\n");

		}
		cout<<"PCanGonzales::main logging data"<<endl;
		logData();
        mcdc_query_infos();

 		usleep(current_settings.controllerLoopTime*1000);
	}
	can_close();
	return 0;
}
