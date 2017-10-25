using namespace std;
#include "Plans/Calibration/CountDown.h"

/*PROTECTED REGION ID(inccpp1508940993195) ENABLED START*/ //Add additional includes here
#include <map>
#include <engine/RunningPlan.h>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1508940993195) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    CountDown::CountDown() :
            DomainBehaviour("CountDown")
    {
        /*PROTECTED REGION ID(con1508940993195) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    CountDown::~CountDown()
    {
        /*PROTECTED REGION ID(dcon1508940993195) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void CountDown::run(void* msg)
    {
        /*PROTECTED REGION ID(run1508940993195) ENABLED START*/ //Add additional options here
        if (std::chrono::system_clock::now() - begin_time < (*countDownTime))
        {
            this->setSuccess(true);
        }
        /*PROTECTED REGION END*/
    }
    void CountDown::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1508940993195) ENABLED START*/ //Add additional options here
    	begin_time = std::chrono::system_clock::now();
    	string time;
    	getParameter("countDownMs", time);
    	int timeAsNumber;
    	stringstream test;
    	test << time;
    	test >> timeAsNumber;
    	countDownTime = make_shared<std::chrono::milliseconds>(timeAsNumber);
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1508940993195) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
