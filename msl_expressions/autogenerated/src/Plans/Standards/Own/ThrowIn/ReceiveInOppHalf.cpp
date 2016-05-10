using namespace std;
#include "Plans/Standards/Own/ThrowIn/ReceiveInOppHalf.h"

/*PROTECTED REGION ID(inccpp1462370340143) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include "SystemConfig.h"
#include "engine/model/EntryPoint.h"
#include "engine/RunningPlan.h"
#include "engine/Assignment.h"
#include "engine/model/Plan.h"
/*PROTECTED REGION END*/
namespace alica
{
	/*PROTECTED REGION ID(staticVars1462370340143) ENABLED START*/ //initialise static variables here
	/*PROTECTED REGION END*/
	ReceiveInOppHalf::ReceiveInOppHalf() :
			DomainBehaviour("ReceiveInOppHalf")
	{
		/*PROTECTED REGION ID(con1462370340143) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	ReceiveInOppHalf::~ReceiveInOppHalf()
	{
		/*PROTECTED REGION ID(dcon1462370340143) ENABLED START*/ //Add additional options here
		/*PROTECTED REGION END*/
	}
	void ReceiveInOppHalf::run(void* msg)
	{
		/*PROTECTED REGION ID(run1462370340143) ENABLED START*/ //Add additional options here
		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData->getOwnPositionVision();
		shared_ptr<geometry::CNPoint2D> egoBallPos = wm->ball->getEgoBallPosition();
		if (ownPos == nullptr || egoBallPos == nullptr)
		{
			return;
		}
		shared_ptr<geometry::CNPoint2D> alloBall = egoBallPos->egoToAllo(*ownPos);
		// Create additional points for path planning
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints = make_shared<
		vector<shared_ptr<geometry::CNPoint2D>>>();
		// add alloBall to path planning
		additionalPoints->push_back(alloBall);


		/*PROTECTED REGION END*/
	}
	void ReceiveInOppHalf::initialiseParameters()
	{
		/*PROTECTED REGION ID(initialiseParameters1462370340143) ENABLED START*/ //Add additional options here



		/*PROTECTED REGION END*/
	}
/*PROTECTED REGION ID(methods1462370340143) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
