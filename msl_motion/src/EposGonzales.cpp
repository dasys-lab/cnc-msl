

#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>
//#include <time.h>
#include "driver/eposcan.h"
#include <udpcanconnection.h>
#include <usbcanconnection.h>

#include "settings.h"
#include "gonzales.h"
#include "RosHelper.h"
#include "logging.h"

#include <string>


#include <SystemConfig.h>
#include <Configuration.h>

#include <getopt.h>
#define CONTROLLER_COUNT 4

using namespace msl_msgs;
using namespace std;
extern gonzales_state gonz_state;

Controlling::EposCan *ep;
CanConnection *cc;

//const char* connection_ipstr;
//int connection_port;

static struct option long_options[] = {
		// option, has arg, flag, val
		{"test", 0, 0, 't'},
		{0, 0, 0, 0}
};

struct timeval time_last;
struct timeval time_cur;



int main(int argc, char** argv) {
	MotionInfo* cmd;
	settings_init();

    SystemConfig* sc = SystemConfig::getInstance();

	Configuration *motion = (*sc)["Motion"];
//printf("Conf: %s\n",motion->get<std::string>("Motion","Connection","IP",NULL).c_str());
	//connection_ipstr = motion->get<std::string>("Motion","Connection","IP",NULL).c_str();
	//connection_port = motion->get<int>("Motion","Connection","Port",NULL);
	
	if (0==motion->get<std::string>("Motion","Connection","Type",NULL).compare("UDP")) {
		printf("Using UDP connection\n");
		UdpCanConnection *udpcc = new UdpCanConnection(motion->get<std::string>("Motion","Connection","IP",NULL).c_str(),motion->get<int>("Motion","Connection","Port",NULL));
		udpcc->SetLocalIP(motion->get<std::string>("Motion","Connection","LocalIP",NULL).c_str());
		cc = udpcc;


	} else if (0==motion->get<std::string>("Motion","Connection","Type",NULL).compare("USB")) {
		printf("Using USB connection\n");
		cc = new UsbCanConnection(motion->get<std::string>("Motion","Connection","Interface",NULL).c_str());
		
 	} else {
		fprintf(stderr, "Invalid connection type in Motion.conf\n");
		exit(-1);
	}	

	ep = new Controlling::EposCan(CONTROLLER_COUNT,cc);


	gonz_init(); //init main controller
    logging_init();


	int option_index = 0;

    char c;
	//char options[4]= "?th";  /* parameterlist for getopt()  */
    while ((c = getopt_long(argc, argv,"?th", long_options, &option_index)) != EOF) {
		switch(c) {
			case 't':
				gonz_set_mode(GONZ_MODE_TEST);
				break;
			case -1: /* error */
			case '?': /* help */
			case 'h': /* help */
				printf("Usage: EposGonzales [-t|--test]\n");
				exit(1);
		}
	}

	cc->Start();
	usleep(30000);


	//if (gonz_get_mode() == GONZ_MODE_NORMAL) {
	printf("Ros Init:\n");
	RosHelper::initialize(argc,argv);
	//}
    //RosHelper::sendInfo("Motion Initialiasing...");
	while(ep->InitAllNodes()<=0 && ros::ok() ) {
        std::cout << "Could not initialise all controllers! Retrying...\n";
		usleep(500000);
	};
    //RosHelper::sendInfo("Motion Initialiased.");
	//unsigned char buffer[5] = {CAN_CMD_SET_VELOCITY,0,0,0,0};
	//INT2BYTEPOS(12*60,buffer,1);
        gonz_state.currentMotionGoal.x = 0;
        gonz_state.currentMotionGoal.y = 0;
        gonz_state.currentMotionGoal.rotation = 0;



	while(ros::ok()) {
	    gettimeofday(&time_last,NULL);
		//char charbuf[256];
	    //writeCanMsg(CAN_ID_PDO2_CMD,1,buffer,5);

        /*
        gonz_state.currentMotionGoal.x = 0;
        gonz_state.currentMotionGoal.y = 0;
        gonz_state.currentMotionGoal.rotation = 0;
        */
		switch(gonz_get_mode()) {
			case GONZ_MODE_NORMAL:
				cmd = RosHelper::getMotion();
                if (cmd!=NULL) {
				    //printf("GOT COMMAND\n");
					gonz_set_motion_request(
						(cmd)->angle,
						(cmd)->translation,
						(cmd)->rotation
					);				
				//printf("MM CURCMD %f\t%f\t%f\n",gonz_state.currentMotionGoal.x,gonz_state.currentMotionGoal.y,gonz_state.currentMotionGoal.rotation);
		       	
					gonz_main();
				} else {
					//printf("Command Timeout!\n");
					gonz_idle();
				}
				break;
			case GONZ_MODE_TEST:
		                gonz_test_loop();
				break;
			default:
				printf("Unknown Mode, doing nothing\n");

		}
		log();

        gettimeofday(&time_cur,NULL);
//printf("Sleeping for: %ldus\n",current_settings.controllerLoopTime*1000l-TIMEDIFFMS(time_cur,time_last));
 		usleep(max(10l,(current_settings.controllerLoopTime-TIMEDIFFMS(time_cur,time_last))*1000l));
	}

	return 0;
}

