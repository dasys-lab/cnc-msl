/*
 * MSLWorldModel.h
 *
 *  Created on: 27.10.2014
 *      Author: Andreas Witsch
 */

#ifndef MSLWORLDMODEL_H_
#define MSLWORLDMODEL_H_

using namespace std;

#include <list>
#include <iostream>
#include <tuple>
#include <mutex>

#include <ros/ros.h>

#include <InformationElement.h>
#include <ITrigger.h>
#include <EventTrigger.h>
#include <MSLEnums.h>

namespace std_msgs
{
	ROS_DECLARE_MESSAGE(Bool)
}
namespace msl_msgs
{
	ROS_DECLARE_MESSAGE(JoystickCommand)
}
namespace msl_actuator_msgs
{
	ROS_DECLARE_MESSAGE(IMUData)
	ROS_DECLARE_MESSAGE(RawOdometryInfo)
	ROS_DECLARE_MESSAGE(MotionBurst)
}
namespace msl_sensor_msgs
{
	ROS_DECLARE_MESSAGE(WorldModelData)
	ROS_DECLARE_MESSAGE(SimulatorWorldModelData)
	ROS_DECLARE_MESSAGE(CorrectedOdometryInfo)
	ROS_DECLARE_MESSAGE(SharedWorldInfo)
	ROS_DECLARE_MESSAGE(BallHypothesisList)
}
namespace gazebo_msgs
{
	ROS_DECLARE_MESSAGE(ModelStates)
}
namespace msl_helper_msgs
{
	ROS_DECLARE_MESSAGE(PassMsg)
	ROS_DECLARE_MESSAGE(WatchBallMsg)
}
namespace alica
{
	class AlicaEngine;
}
namespace supplementary
{
	class SystemConfig;
}
namespace geometry
{
	class CNPoint2D;
	class CNPosition;
}

namespace msl
{
	class RawSensorData;
	class Robots;
	class Ball;
	class Game;
	class WhiteBoard;
	class MSLFootballField;
	class Prediction;
	class Monitoring;
	class LightBarrier;
	class Obstacles;
	class PathPlanner;
	class MSLSharedWorldModel;
	class MSLWorldModel
	{
	public:
		static MSLWorldModel* get();
		bool setEngine(alica::AlicaEngine* ae);
		alica::AlicaEngine* getEngine();

		void onRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg);
		void onBallHypothesisList(msl_sensor_msgs::BallHypothesisListPtr msg);
		void onWorldModelData(msl_sensor_msgs::WorldModelDataPtr msg);
		void onJoystickCommand(msl_msgs::JoystickCommandPtr msg);
		void onMotionBurst(msl_actuator_msgs::MotionBurstPtr msg);
		void onSimWorldModel(msl_sensor_msgs::SimulatorWorldModelDataPtr msg);
		void onGazeboModelState(gazebo_msgs::ModelStatesPtr msg);
		void onSharedWorldInfo(msl_sensor_msgs::SharedWorldInfoPtr msg);
		void onPassMsg(msl_helper_msgs::PassMsgPtr msg);
		void onWatchBallMsg(msl_helper_msgs::WatchBallMsgPtr msg);
		void onCorrectedOdometryInfo(msl_sensor_msgs::CorrectedOdometryInfoPtr msg);
		void onLightBarrierInfo(std_msgs::BoolPtr msg);
		void onIMUData(msl_actuator_msgs::IMUDataPtr msg);

		bool isMaySendMessages() const;
		void setMaySendMessages(bool maySendMessages);

		MSLSharedWorldModel* getSharedWorldModel();
		InfoTime getTime();
		void sendSharedWorldModelData();

		int getRingBufferLength();
		int getOwnId();
		supplementary::ITrigger* getVisionDataEventTrigger();

		bool isUsingSimulator();

		Monitoring* monitoring;
		RawSensorData* rawSensorData;
		Robots* robots;
		Ball* ball;
		Game* game;
		MSLFootballField* field;
		PathPlanner* pathPlanner;
		WhiteBoard* whiteBoard;
		Obstacles* obstacles;
		Prediction* prediction;
		LightBarrier* lightBarrier;

		supplementary::EventTrigger visionTrigger;
		InfoTime timeLastSimMsgReceived;
	private:

		MSLWorldModel();
		virtual ~MSLWorldModel();

		supplementary::ITrigger* visionDataEventTrigger;
		supplementary::SystemConfig* sc;

		int ownID;
		int ringBufferLength;
		bool maySendMessages;

		MSLSharedWorldModel* sharedWorldModel;
		alica::AlicaEngine* alicaEngine;

		ros::NodeHandle n;
		ros::Subscriber sub;
		ros::Subscriber rawOdomSub;
		ros::Subscriber wmDataSub;
		ros::Subscriber wmBallListSub;
		ros::Subscriber joystickSub;
		ros::Subscriber motionBurstSub;
		ros::Subscriber simWorldModelSub;
		ros::Subscriber gazeboWorldModelSub;
		ros::Subscriber sharedWorldSub;
		ros::Subscriber passMsgSub;
		ros::Subscriber watchBallMsgSub;
		ros::Publisher sharedWorldPub;
		ros::Subscriber correctedOdometrySub;
		ros::Subscriber lightBarrierSub;
		ros::Subscriber imuDataSub;

		list<msl_msgs::JoystickCommandPtr> joystickCommandData;

		mutex wmMutex;
		mutex joystickMutex;
		mutex motionBurstMutex;
		mutex correctedOdemetryMutex;
		ros::AsyncSpinner* spinner;

	protected:

	};

} /* namespace msl */

#endif /* MSLWORLDMODEL_H_ */
