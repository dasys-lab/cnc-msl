#include "logging.h"

int isLogging = 0;
std::string logFile;
FILE *lp;
extern Controlling::EposCan *ep;

void logging_init() {
    SystemConfig* sc;
   	sc = SystemConfig::getInstance();

	Configuration *motion = (*sc)["Motion"];
	isLogging = motion->get<bool>("Motion","Logging","LogStuff",NULL);
    if (isLogging) {
        logFile = motion->get<std::string>("Motion","Logging","LogFile",NULL);
        lp = fopen(logFile.c_str(),"a");
        if (lp==NULL) {
            printf("Cannot Open Log File!\n");
            exit(-1);
        }
    }
}

void logData() {
	cout<<"logging::log was called"<<endl;
    if (isLogging) {
    	cout<<"logging::log isLogging in file "<<logFile<<endl;
        fprintf(lp,"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%f\n",
            gonz_state.currentPosition.angle,gonz_state.currentPosition.x,gonz_state.currentPosition.y,
            gonz_state.actualMotion.x,gonz_state.actualMotion.y,gonz_state.actualMotion.rotation,
            gonz_state.currentMotionGoal.x,gonz_state.currentMotionGoal.y,gonz_state.currentMotionGoal.rotation,
			ep->ActualRPM(0),ep->ActualRPM(1),ep->ActualRPM(2),ep->ActualRPM(3),
			ep->DemandRPM(0),ep->DemandRPM(1),ep->DemandRPM(2),ep->DemandRPM(3),
			ep->ActualCurrent(0),ep->ActualCurrent(1),ep->ActualCurrent(2),ep->ActualCurrent(3),
			gonz_state.currentSlip
        );

    }
}
