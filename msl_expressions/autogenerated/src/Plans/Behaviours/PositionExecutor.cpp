using namespace std;
#include "Plans/Behaviours/PositionExecutor.h"

/*PROTECTED REGION ID(inccpp1438790362133) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include "engine/model/EntryPoint.h"
#include "engine/RunningPlan.h"
#include "engine/Assignment.h"
#include "engine/model/Plan.h"
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1438790362133) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	PositionExecutor::PositionExecutor() :
			DomainBehaviour("PositionExecutor")
	{
		/*PROTECTED REGION ID(con1438790362133) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	PositionExecutor::~PositionExecutor()
	{
		/*PROTECTED REGION ID(dcon1438790362133) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void PositionExecutor::run(void* msg)
	{
		/*PROTECTED REGION ID(run1438790362133) ENABLED START*/ //Add additional options here
		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData.getOwnPositionVision(); // actually ownPosition corrected
		shared_ptr<geometry::CNPoint2D> egoBallPos = wm->ball.getEgoBallPosition();

		// return if necessary information is missing
		if (ownPos == nullptr || egoBallPos == nullptr)
		{
			return;
		}

		// Create allo ball
		shared_ptr<geometry::CNPoint2D> alloBall = egoBallPos->egoToAllo(*ownPos);

		// Create additional points for path planning
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<
		vector<shared_ptr<geometry::CNPoint2D>>>();
		// add alloBall to path planning
		additionalPoints->push_back(alloBall);

		// get entry point of task name to locate robot with task name
		EntryPoint* ep = getParentEntryPoint(taskName);
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
			shared_ptr<geometry::CNPoint2D> receiverPos = nullptr;
			// exactly one robot is receiver
			int id = ids->at(0);
			if (id != -1)
			{
				// get receiver position by id
				auto pos = wm->robots.getTeamMatePosition(id);
				receiverPos = make_shared<geometry::CNPoint2D>(pos->x, pos->y);
			}
			MotionControl mc;
			shared_ptr<geometry::CNPoint2D> egoTarget = nullptr;
			// if there is a receiver, align to it
			if (receiverPos != nullptr)
			{
				// calculate target 60cm away from the ball and on a line with the receiver
				egoTarget = (alloBall + ((alloBall - receiverPos)->normalize() * 600))->alloToEgo(*ownPos);
				// ask the path planner how to get there
				mc = msl::RobotMovement::moveToPointCarefully(egoTarget, receiverPos->alloToEgo(*ownPos), 0,
															additionalPoints);
			}
			else
			{
				// if there is no receiver, align to middle
				egoTarget = (alloBall + ((alloBall - alloTarget)->normalize() * 600))->alloToEgo(*ownPos);
				mc = msl::RobotMovement::moveToPointCarefully(egoTarget, alloTarget->alloToEgo(*ownPos), 0,
															additionalPoints);
			}
			// if we reach the point and are aligned, the behavior is successful
			if (egoTarget->length() < 250 && fabs(egoBallPos->rotate(M_PI)->angleTo()) < (M_PI / 180) * 5)
			{
				this->success = true;
			}
			send(mc);
		}
		/*PROTECTED REGION END*/
	}
	void PositionExecutor::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1438790362133) ENABLED START*/ //Add additional options here
		string receiverTaskName;
		if (getParameter("receiverTask", receiverTaskName))
		{
			EntryPoint* receiverEp = this->getParentEntryPoint(receiverTaskName);
			if (receiverEp == nullptr)
			{ // there is no entrypoint with the given receiver task attached
			  // repair that stuff with getparent and weak pointer ... see run method
				auto parent = this->runningPlan->getParent().lock();
				if (parent != nullptr &&((Plan*)parent->getPlan())->getEntryPoints().size() == 2)
				{ // there is only one other entry point than our own entry point, so it must be the receivers entry point.
				  // TODO which is my own entry point, so take the other one for the receiver
				  //this->runningPlan->getActiveEntryPoint()
					auto activeEp = this->runningPlan->getActiveEntryPoint();
					auto eps = ((Plan*)parent->getPlan())->getEntryPoints();
				}
				else
				{
					cerr << "PositionExecutor: Could not determine the receivers entry point!" << endl;
				}
			}
			else
			{ // we found the entry point of the receiver, so everything is cool

			}
		}
		else
		{
			cerr << "PositionExecutor: Parameter receiverTask does not exists!" << endl;
		}

		field = msl::MSLFootballField::getInstance();
		string tmp;
		bool success = true;
		alloTarget = make_shared<geometry::CNPoint2D>(0, 0);
		success &= getParameter("TaskName", tmp);
		try
		{
			if (success)
			{
				taskName = tmp;
			}
		}
		catch (exception& e)
		{
			cerr << "Could not cast the parameter properly" << endl;
		}
		if (!success)
		{
			cerr << "Parameter does not exist" << endl;
		}
		/*PROTECTED REGION END*/
	}
/*PROTECTED REGION ID(methods1438790362133) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
