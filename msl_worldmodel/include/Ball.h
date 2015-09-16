/*
 * Ball.h
 *
 *  Created on: Feb 24, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_BALL_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_BALL_H_

#include "container/CNPoint2D.h"
#include "container/CNVelocity2D.h"
#include "RingBuffer.h"
#include "InformationElement.h"
#include <map>
#include <memory>
#include "msl_sensor_msgs/SharedWorldInfo.h"

#include "SystemConfig.h"

using namespace supplementary;

using namespace std;

namespace msl
{

	class MSLWorldModel;
	class Ball
	{
	public:
		Ball(MSLWorldModel* wm);
		virtual ~Ball();
		bool haveBall();
		shared_ptr<geometry::CNPoint2D> getAlloBallPosition();
		shared_ptr<geometry::CNPoint2D> getEgoBallPosition();
		shared_ptr<geometry::CNPoint2D> getEgoRawBallPosition();
		shared_ptr<geometry::CNVelocity2D> getEgoBallVelocity();
		void updateOnWorldModelData();
		void processSharedWorldModelData(msl_sensor_msgs::SharedWorldInfo data);
		shared_ptr<bool> getTeamMateBallPossession(int teamMateId, int index = 0);
		shared_ptr<bool> getOppBallPossession(int index = 0);
		shared_ptr<geometry::CNPoint2D> getSharedBallPosition();
		double getBallDiameter();

	private:
		shared_ptr<geometry::CNPoint2D> lastKnownBallPos;
		shared_ptr<geometry::CNPoint2D> sharedBallPosition;
		double HAVE_BALL_TOLERANCE_DRIBBLE;
		double KICKER_DISTANCE;
		double KICKER_ANGLE;
		double HAVE_BALL_MAX_ANGLE_DELTA;
		double BALL_DIAMETER;
		int hasBallIteration;
		bool hadBefore;
		bool hasBall; /**< True if the local robot has the ball */
		double haveBallDistanceDynamic;
		unsigned long maxInformationAge = 1000000000;
		MSLWorldModel* wm;
		SystemConfig* sc;
		map<int, shared_ptr<RingBuffer<InformationElement<bool>>>> ballPossession;
		shared_ptr<RingBuffer<InformationElement<bool>>> oppBallPossession;
		map<int, shared_ptr<RingBuffer<InformationElement<geometry::CNPoint2D>>>> ballPositionsByRobot;
		map<int, shared_ptr<RingBuffer<InformationElement<geometry::CNVelocity2D>>>> ballVelocitiesByRobot;
		bool robotHasBall(int robotId);
		bool oppHasBall(msl_sensor_msgs::SharedWorldInfo data);
};

} /* namespace alica */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_BALL_H_ */
