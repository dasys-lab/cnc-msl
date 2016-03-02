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
#include "ballTracking/ObjectContainer.h"
#include "ballTracking/TrackingTypes.h"
#include "SystemConfig.h"

using namespace supplementary;

using namespace std;

namespace msl
{


	class BallVoting
	{
	public:
		shared_ptr<geometry::CNPoint2D> ballPos;
		double teamConfidence = 0.0;
		vector<shared_ptr<msl_sensor_msgs::SharedWorldInfo>> supporters;
	};


	class MSLWorldModel;
	class Ball
	{
	public:
		Ball(MSLWorldModel* wm, int ringbufferLength);
		virtual ~Ball();
		bool haveBall();
		bool haveBallDribble(bool hadBefore);

		shared_ptr<geometry::CNPoint2D> getVisionBallPosition(int index = 0);
		shared_ptr<pair<shared_ptr<geometry::CNPoint2D>, double>> getVisionBallPositionAndCertaincy(int index = 0);
		shared_ptr<geometry::CNVelocity2D> getVisionBallVelocity(int index = 0);
		double getBallConfidenceVision(int index = 0);

		shared_ptr<geometry::CNPoint2D> getAlloBallPosition();
		shared_ptr<geometry::CNPoint2D> getEgoBallPosition();
		shared_ptr<geometry::CNVelocity2D> getEgoBallVelocity();
		bool ballMovedSiginficantly();


		void updateHaveBall();
		void updateOnBallHypothesisList(unsigned long long imageTime);
		void updateOnLocalizationData(unsigned long long imageTime);
		void processHypothesis();
		void updateBallPos(shared_ptr<geometry::CNPoint2D> ballPos, shared_ptr<geometry::CNVelocity2D> ballVel, double certainty);
		void processSharedWorldModelData(msl_sensor_msgs::SharedWorldInfo data);
		shared_ptr<bool> getTeamMateBallPossession(int teamMateId, int index = 0);
		shared_ptr<bool> getOppBallPossession(int index = 0);
		shared_ptr<geometry::CNPoint2D> getAlloSharedBallPosition();
		double getBallDiameter();

		void updateSharedBall();
		double calculateSharedBallMassVector(bool withGoalie);
		bool simpleHaveBallDribble(bool hadBefore);
		bool hadBefore;

	private:
		std::mutex sbMutex;
		vector<BallVoting> sbvotingList;
		int sharedBallSupporters;
		double sharedBallTeamConfidence;
		shared_ptr<geometry::CNPoint2D> sharedBallPosition;

		ObjectContainer ballBuf;
		MovingObject mv;
		unsigned long long lastUpdateReceived;
		shared_ptr<geometry::CNPoint2D> lastKnownBallPos;
		double HAVE_BALL_TOLERANCE_DRIBBLE;
		double KICKER_DISTANCE;
		double KICKER_DISTANCE_SIMULATOR;
		double KICKER_ANGLE;
		double HAVE_BALL_MAX_ANGLE_DELTA;
		double BALL_DIAMETER;
		double LOCALIZATION_SUCCESS_CONFIDENCE;
		int hasBallIteration;
		bool hasBall; /**< True if the local robot has the ball */
		double haveBallDistanceDynamic;
		unsigned long maxInformationAge = 1000000000;
		MSLWorldModel* wm;
		SystemConfig* sc;
		map<int, shared_ptr<RingBuffer<InformationElement<bool>>>> ballPossession;
		RingBuffer<InformationElement<bool>> oppBallPossession;
		map<int, shared_ptr<RingBuffer<InformationElement<geometry::CNPoint2D>>>> ballPositionsByRobot;
		map<int, shared_ptr<RingBuffer<InformationElement<geometry::CNVelocity2D>>>> ballVelocitiesByRobot;
		RingBuffer<InformationElement<geometry::CNPoint2D>> ballPosition;
		RingBuffer<InformationElement<geometry::CNVelocity2D>> ballVelocity;
		bool robotHasBall(int robotId);
		bool oppHasBall(msl_sensor_msgs::SharedWorldInfo data);
		Point allo2Ego(Point p, Position pos);
		Velocity allo2Ego(Velocity vel, Position pos);
		double haveDistance;
};


} /* namespace alica */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_BALL_H_ */
