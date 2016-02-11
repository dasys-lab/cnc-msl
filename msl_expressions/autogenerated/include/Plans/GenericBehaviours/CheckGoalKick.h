#ifndef CheckGoalKick_H_
#define CheckGoalKick_H_

#include "DomainBehaviour.h"
/*PROTECTED REGION ID(inc1449076008755) ENABLED START*/ //Add additional includes here
/*PROTECTED REGION END*/
namespace alica
{
	class CheckGoalKick : public DomainBehaviour
	{
	public:
		CheckGoalKick();
		virtual ~CheckGoalKick();
		virtual void run(void* msg);
		/*PROTECTED REGION ID(pub1449076008755) ENABLED START*/ //Add additional public methods here
		/*PROTECTED REGION END*/
	protected:
		virtual void initialiseParameters();
		/*PROTECTED REGION ID(pro1449076008755) ENABLED START*/ //Add additional protected methods here
		/*PROTECTED REGION END*/
	private:
		/*PROTECTED REGION ID(prv1449076008755) ENABLED START*/ //Add additional private methods here
		shared_ptr<geometry::CNPoint2D> ownPos;
		shared_ptr<geometry::CNPoint2D> egoBallPos;
		shared_ptr<geometry::CNPoint2D> goalPosLeft;
		shared_ptr<geometry::CNPoint2D> goalPosRight;
		shared_ptr<geometry::CNPoint2D> goalPosMiddle;
		shared_ptr<geometry::CNPoint2D> egoAlignPoint;
//		shared_ptr<geometry::CNPoint2D> alloTargetPoint;
//		shared_ptr<geometry::CNPoint2D> egoTargetPoint;
		shared_ptr<geometry::CNPoint2D> alloLeftAimPoint;
		shared_ptr<geometry::CNPoint2D> alloRightAimPoint;
		shared_ptr<geometry::CNPoint2D> alloMidAimPoint;
		msl::MSLFootballField* field;
		double minObsDistGoal;
		double minOwnDistGoal;
		double closeGoalDist;
		double farGoalDist;
		double obsDistGoal;
		double ownDistGoal;
		double ownDistObs;
		double keeperDistGoal;
		double toleranceAngle;
		double minKickPower;
		double waitingIter;
		double wheelSpeedLeft;
		double wheelSpeedRight;
		bool checkGoalLine();
		bool checkShootPossibility();
		void readConfigParameters();
		double calcToleranceAngle();
		void kicking();
		bool checkGoalKeeper();
		shared_ptr<vector<geometry::CNPoint2D>> getObstacles();
		/*PROTECTED REGION END*/};
} /* namespace alica */

#endif /* CheckGoalKick_H_ */