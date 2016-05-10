using namespace std;
#include "Plans/GenericStandards/StandardAlignAndGrab2Receivers.h"

/*PROTECTED REGION ID(inccpp1462368682104) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include "engine/model/EntryPoint.h"
#include "engine/RunningPlan.h"
#include "engine/Assignment.h"
#include "engine/model/Plan.h"
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1462368682104) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	StandardAlignAndGrab2Receivers::StandardAlignAndGrab2Receivers() :
			DomainBehaviour("StandardAlignAndGrab2Receivers")
	{
		/*PROTECTED REGION ID(con1462368682104) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	StandardAlignAndGrab2Receivers::~StandardAlignAndGrab2Receivers()
	{
		/*PROTECTED REGION ID(dcon1462368682104) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void StandardAlignAndGrab2Receivers::run(void* msg)
	{
		/*PROTECTED REGION ID(run1462368682104) ENABLED START*/ //Add additional options here
		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData->getOwnPositionVision(); // actually ownPosition corrected
		shared_ptr<geometry::CNPoint2D> egoBallPos = wm->ball->getEgoBallPosition();
		// return if necessary information is missing
		if (ownPos == nullptr || egoBallPos == nullptr)
		{
			return;
		}
		shared_ptr<geometry::CNPoint2D> alloBall = egoBallPos->egoToAllo(*ownPos);

		EntryPoint* ep = getParentEntryPoint(teamMateTaskName1);
		if (ep != nullptr)
		{
			// get the plan in which the behavior is running
			auto parent = this->runningPlan->getParent().lock();
			if (parent == nullptr)
			{
				cout << "parent null" << endl;
				return;
			}
			// get robot ids of robots in found entry point
			shared_ptr<vector<int>> ids = parent->getAssignment()->getRobotsWorking(ep);
			// exactly one robot is receiver
			if (ids->size() > 0 && ids->at(0) != -1)
			{
				// get receiver position by id
				auto pos = wm->robots->teammates.getTeamMatePosition(ids->at(0));
				if (pos != nullptr)
				{
					recPos1 = make_shared<geometry::CNPoint2D>(pos->x, pos->y);
				}
				else
				{
					recPos1 = nullptr;
				}
			}
		}

		EntryPoint* ep2 = getParentEntryPoint(teamMateTaskName2);
		if (ep2 != nullptr)
		{
			// get the plan in which the behavior is running
			auto parent = this->runningPlan->getParent().lock();
			if (parent == nullptr)
			{
				cout << "parent null" << endl;
				return;
			}
			// get robot ids of robots in found entry point
			shared_ptr<vector<int>> ids = parent->getAssignment()->getRobotsWorking(ep2);
			// exactly one robot is receiver
			if (ids->size() > 0 && ids->at(0) != -1)
			{
				// get receiver position by id
				auto pos = wm->robots->teammates.getTeamMatePosition(ids->at(0));
				if (pos != nullptr)
				{
					recPos2 = make_shared<geometry::CNPoint2D>(pos->x, pos->y);
				}
				else
				{
					recPos2 = nullptr;
				}
			}
		}



		MotionControl mc;

		/*PROTECTED REGION END*/
	}
	void StandardAlignAndGrab2Receivers::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1462368682104) ENABLED START*/ //Add additional options here
		string tmp;
		bool success = true;
		try
		{
			success &= getParameter("TeamMateTaskName1", tmp);
			if (success)
			{
				teamMateTaskName1 = tmp;
			}

			success &= getParameter("TeamMateTaskName", tmp);
			if (success)
			{
				teamMateTaskName2 = tmp;
			}

		}
		catch (exception& e)
		{
			cerr << "Could not cast the parameter properly" << endl;
		}
		if (!success)
		{
			cerr << "SA2P: Parameter does not exist" << endl;
		}

		/*PROTECTED REGION END*/
	}
/*PROTECTED REGION ID(methods1462368682104) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
