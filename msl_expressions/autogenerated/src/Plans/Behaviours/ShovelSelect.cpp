using namespace std;
#include "Plans/Behaviours/ShovelSelect.h"

/*PROTECTED REGION ID(inccpp1434199834892) ENABLED START*/ //Add additional includes here
#include "msl_actuator_msgs/ShovelSelectCmd.h"
#include "engine/RunningPlan.h"
#include "engine/Assignment.h"
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
        last = 0;
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
        shared_ptr < geometry::CNPosition > ownPos = wm->rawSensorData.getOwnPositionVision();
        if (ownPos == nullptr)
        {
            return;
        }
        //int mateId = 0;
        shared_ptr < geometry::CNPosition > matePos = nullptr;

        shared_ptr<vector<int>> robots = getRunningPlan()->getAssignment()->getRobotsWorking(
                getRunningPlan()->getOwnEntryPoint());

        for (int i = 0; i < robots->size(); i++)
        {
            //TODO expand shared world model
//			matePos = wm->
            break;
        }

        if (matePos != nullptr)
        {

            double passDist = sqrt(pow(ownPos->x - matePos->x, 2) + pow(ownPos->y - matePos->y, 2));

            if (passDist > 4000)
            {
                ssc.passing = false;

            }
            else
            {
                ssc.passing = true;
            }
        }

        long now = wm->getTime();
        long diff = now - last;
        if (last == 0 || diff > 5000000)
        {
            send(ssc);
            last = now;
        }
        /*PROTECTED REGION END*/
    }
    void ShovelSelect::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1434199834892) ENABLED START*/ //Add additional options here
        string tmp;
        bool success = true;
        success &= getParameter("passing", tmp);
        try
        {
            if (success)
            {
                std::transform(tmp.begin(), tmp.end(), tmp.begin(), ::tolower);
                std::istringstream is(tmp);
                is >> std::boolalpha >> passing;
            }

        }
        catch (exception& e)
        {
            cerr << "Could not cast the parameter properly" << endl;
        }
        if (!success)
        {
            cerr << "Parameter does not exist Passing" << endl;
        }
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1434199834892) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
