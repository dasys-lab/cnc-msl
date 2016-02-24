/*
 * MSLWorldModel.h
 *
 *  Created on: 27.10.2014
 *      Author: Andreas Witsch
 */

#ifndef MSLWORLDMODEL_H_
#define MSLWORLDMODEL_H_

#include <ros/ros.h>
#include "msl_actuator_msgs/RawOdometryInfo.h"
#include <msl_sensor_msgs/WorldModelData.h>
#include <msl_msgs/JoystickCommand.h>
#include <msl_actuator_msgs/MotionBurst.h>
#include <msl_sensor_msgs/SimulatorWorldModelData.h>
#include <gazebo_msgs/ModelStates.h>
#include <msl_helper_msgs/PassMsg.h>
#include <msl_sensor_msgs/CorrectedOdometryInfo.h>
#include <msl_sensor_msgs/BallHypothesisList.h>
#include <list>
#include <iostream>
#include <tuple>
#include <mutex>

#include <SystemConfig.h>
#include <container/CNPoint2D.h>
#include <container/CNPosition.h>
#include <MSLEnums.h>
#include "RawSensorData.h"
#include "Robots.h"
#include "Ball.h"
#include "Game.h"
#include "Kicker.h"
#include "WhiteBoard.h"
#include "pathplanner/PathPlanner.h"
#include "EventTrigger.h"
#include "InformationElement.h"
#include "obstaclehandler/ObHandler.h"

namespace alica {
	class AlicaEngine;
}

using namespace std;

namespace msl
{


	class MSLSharedWorldModel;
	class MSLWorldModel
	{
	public:
		static MSLWorldModel* get();
		bool setEngine(alica::AlicaEngine* ae);
		alica::AlicaEngine* getEngine();

		double getKickerVoltage();
		void setKickerVoltage(double voltage);


		void onRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg);
		void onBallHypothesisList(msl_sensor_msgs::BallHypothesisListPtr msg);
		void onWorldModelData(msl_sensor_msgs::WorldModelDataPtr msg);
		void onJoystickCommand(msl_msgs::JoystickCommandPtr msg);
		void onMotionBurst(msl_actuator_msgs::MotionBurstPtr msg);
		void onSimWorldModel(msl_sensor_msgs::SimulatorWorldModelDataPtr msg);
		void onGazeboModelState(gazebo_msgs::ModelStatesPtr msg);
		void onSharedWorldInfo(msl_sensor_msgs::SharedWorldInfoPtr msg);
		void onPassMsg(msl_helper_msgs::PassMsgPtr msg);
		void onCorrectedOdometryInfo(msl_sensor_msgs::CorrectedOdometryInfoPtr msg);


		MSLSharedWorldModel* getSharedWorldModel();
		InfoTime getTime();
		void sendSharedWorldModelData();


		int getRingBufferLength();
		int getOwnId();

		RawSensorData rawSensorData;
		Robots robots;
		Ball ball;
		Game game;
		PathPlanner pathPlanner;
		Kicker kicker;
		WhiteBoard whiteBoard;
		supplementary::EventTrigger visionTrigger;
		InfoTime timeLastSimMsgReceived;
		ObHandler obstacleHandler;


	private:

		MSLWorldModel();
		virtual ~MSLWorldModel();

		int ownID;
		int ringBufferLength;
		double kickerVoltage;
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
		ros::Publisher sharedWorldPub;
		ros::Subscriber correctedOdometrySub;

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
