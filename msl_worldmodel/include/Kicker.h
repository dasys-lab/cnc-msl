/*
 * Kicker.h
 *
 *  Created on: Jul 13, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_KICKER_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_KICKER_H_
#include <memory>
#include "GeometryCalculator.h"
#include "SystemConfig.h"
#include "KickCurve.h"

using namespace std;
namespace msl
{

	class MSLWorldModel;
	class Kicker
	{
	public:
		Kicker(MSLWorldModel* wm);
		virtual ~Kicker();
		bool init();
		int getKickPowerPass(double dist);
		int getKickPowerSlowPass(double dist);
		int getKickerCount();
		int getKickPower(double dist, double height, double velo);
		bool mayShoot();
		shared_ptr<geometry::CNPoint2D> getFreeGoalVector();
		double minFree(double angle, double width, shared_ptr<vector<double>> dstscan);
		double getPassKickpower(double dist, double arrivalTime);
		double getPassVelocity(double kickpower);
		double getPreciseShotMaxDistance();
		double getPreciseShotMaxTolerance();
		double getPreciseShotMinDistance();
		int getShortPassPower();
		bool lowShovelSelected;
		static double kickerAngle;

	private:
		MSLWorldModel* wm;
	protected:
		supplementary::SystemConfig* sc ;
		double TWO_THIRD_PI;
		int kickerCount;
		double ballisticCurveOffset;
		double minVoltage;
		double maxVoltage;
		double powerMult;

		double preciseShotMaxDistance;
		double preciseShotMinDistance;
		double preciseShotMaxTolerance;
		int shortPassPower;

		KickCurve* kick2GoalCurve;
		KickCurve* kickHighPass;
		KickCurve* kickLowPass;
		vector<shared_ptr<geometry::CNPoint2D>> validGoalPoints ;
		int mod(int x, int y);
	};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_KICKER_H_ */
