#include "logging.h"

bool isLogging = false;
std::string fileName;
FILE *logFile;
extern Controlling::EposCan *ep;

void logging_init()
{
    SystemConfig *sc = SystemConfig::getInstance();
    isLogging = (*sc)["Motion"]->get<bool>("Motion", "Logging", "LogStuff", NULL);
    if (isLogging)
    {
        fileName = (*sc)["Motion"]->get<std::string>("Motion", "Logging", "LogFile", NULL);
        logFile = fopen(fileName.c_str(), "a");
        if (logFile == NULL)
        {
            printf("Cannot Open Log File!\n");
            exit(-1);
        }
    }
}

void logData()
{
    if (isLogging)
    {
        fprintf(logFile, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%f\n", gonz_state.currentPosition.angle,
                gonz_state.currentPosition.x, gonz_state.currentPosition.y, gonz_state.actualMotion.x, gonz_state.actualMotion.y,
                gonz_state.actualMotion.rotation, gonz_state.currentMotionGoal.x, gonz_state.currentMotionGoal.y, gonz_state.currentMotionGoal.rotation,
                ep->ActualRPM(0), ep->ActualRPM(1), ep->ActualRPM(2), ep->ActualRPM(3), ep->DemandRPM(0), ep->DemandRPM(1), ep->DemandRPM(2), ep->DemandRPM(3),
                ep->ActualCurrent(0), ep->ActualCurrent(1), ep->ActualCurrent(2), ep->ActualCurrent(3), gonz_state.currentSlip);
    }
}
