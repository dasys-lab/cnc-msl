using namespace std;
#include "Plans/Behaviours/ShovelSelect.h"

/*PROTECTED REGION ID(inccpp1434199834892) ENABLED START*/ //Add additional includes here
#include "msl_actuator_msgs/ShovelSelectCmd.h"
#include "engine/RunningPlan.h"
#include "engine/Assignment.h"

#include <iostream>
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1434199834892) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    ShovelSelect::ShovelSelect() :
            DomainBehaviour("ShovelSelect")
    {
        /*PROTECTED REGION ID(con1434199834892) ENABLED START*/ //Add additional options here
        passing = false;
        /*PROTECTED REGION END*/
    }
    ShovelSelect::~ShovelSelect()
    {
        /*PROTECTED REGION ID(dcon1434199834892) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void ShovelSelect::run(void* msg)
    {
        /*PROTECTED REGION ID(run1434199834892) ENABLED START*/ //Add additional options here
        msl_actuator_msgs::ShovelSelectCmd ssc = msl_actuator_msgs::ShovelSelectCmd();
        ssc.passing = this->passing;
        send(ssc);
/*        auto lb = wm->rawSensorData.getLightBarrier();
        if (lb)
        {
	    if(*lb)
	    {
                std::cout << "Hab den Ball :)" << std::endl;
	    }
	    else
	    {
		std::cout << "Hab ihn nicht. :(" << std::endl;
	    }
        }
        else
        {
            std::cout << "NullPtr :(" << std::endl;
        }*/
        /*PROTECTED REGION END*/
    }
    void ShovelSelect::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1434199834892) ENABLED START*/ //Add additional options here
        string tmp;
        try
        {
            if (getParameter("passing", tmp))
            {
                std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::tolower);
                std::istringstream is(tmp);
                is >> std::boolalpha >> passing;
            }
            else
            {
                cerr << "Parameter does not exist Passing" << endl;
            }

        }
        catch (exception& e)
        {
            cerr << "Could not cast the parameter properly" << endl;
        }
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1434199834892) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
