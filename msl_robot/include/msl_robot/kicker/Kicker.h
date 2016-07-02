/*
 * Kicker.h
 *
 *  Created on: Jul 13, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_KICKER_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_KICKER_H_

#include "KickCurve.h"

#include <memory>

#include <SystemConfig.h>
#include <msl_actuator_msgs/KickControl.h>
#include <RingBuffer.h>
#include <InformationElement.h>

using namespace std;

namespace geometry{
	class CNPoint2D;
}
namespace msl
{

	class MSLWorldModel;
	class Kicker
	{
	public:
		Kicker(MSLWorldModel* wm);
		virtual ~Kicker();
		bool init();
		float getKickerVoltage();
		void setKickerVoltage(float kickerVoltage);
		int getKickPowerPass(double dist);
		int getKickPowerSlowPass(double dist);
		int getKickerCount();
		int getKickPower(double dist, double height, double velo);
		bool mayShoot();
		shared_ptr<geometry::CNPoint2D> getFreeGoalVector();
		double minFree(double angle, double width, shared_ptr<vector<double>> dstscan);
		double getPassKickpower(double dist, double arrivalTime);
		double getPassVelocity(double kickpower);
		double getKickPowerForLobShot(double dist, double height, double heightTolerance = 30.0);
		double getPreciseShotMaxDistance();
		double getPreciseShotMaxTolerance();
		double getPreciseShotMinDistance();
		int getShortPassPower();
		bool lowShovelSelected;
		static double kickerAngle;

		void processKickConstrolMsg(msl_actuator_msgs::KickControl& km);
		shared_ptr<msl_actuator_msgs::KickControl> getKickConstrolMsg(int index = 0);

	private:
		MSLWorldModel* wm;
		float kickerVoltage;
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

		double kickerAngleOffset;

		KickCurve* kick2GoalCurve;
		KickCurve* kickHighPass;
		KickCurve* kickLowPass;
		vector<shared_ptr<geometry::CNPoint2D>> validGoalPoints ;

		RingBuffer<InformationElement<msl_actuator_msgs::KickControl>> kickControlMsgs;
		int mod(int x, int y);
	};

} /* namespace msl */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_KICKER_H_ */
