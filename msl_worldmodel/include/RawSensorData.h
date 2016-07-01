

/*
 * RawSensorData.h
 *
 *  Created on: Feb 18, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_RAWSENSORDATA_H_
#define CNC_MSL_MSL_WORLDMODEL_RAWSENSORDATA_H_

#include "InformationElement.h"
#include <vector>
#include "msl_sensor_msgs/BallInfo.h"
#include "msl_msgs/MotionInfo.h"
#include "msl_actuator_msgs/MotionControl.h"
#include "msl_msgs/JoystickCommand.h"
#include "msl_actuator_msgs/MotionBurst.h"
#include "msl_actuator_msgs/HaveBallInfo.h"
#include "container/CNPosition.h"
#include "container/CNVelocity2D.h"
#include "container/CNPoint2D.h"
#include "msl_sensor_msgs/WorldModelData.h"
#include "msl_sensor_msgs/BallHypothesisList.h"
#include "msl_actuator_msgs/RawOdometryInfo.h"
#include "std_msgs/Bool.h"
#include "msl_actuator_msgs/IMUData.h"
#include "RingBuffer.h"

using namespace std;

namespace msl
{

	class MSLWorldModel;
	class RawSensorData
	{
	public:
		RawSensorData(MSLWorldModel* wm, int ringBufferLength);
		virtual ~RawSensorData();
		shared_ptr<vector<double>> getDistanceScan(int index = 0);
		bool getLightBarrier(int index = 0);
		shared_ptr<geometry::CNPoint2D> getOpticalFlow(int index = 0);
		double getOpticalFlowQoS(int index = 0);
		shared_ptr<geometry::CNPosition> getOwnPositionMotion(int index = 0);
		shared_ptr<geometry::CNPosition> getOwnPositionVision(int index = 0);
		shared_ptr<pair<shared_ptr<geometry::CNPosition>, double>> getOwnPositionVisionAndCertaincy(int index = 0);
		shared_ptr<msl_msgs::MotionInfo> getOwnVelocityMotion(int index = 0);
		shared_ptr<msl_msgs::MotionInfo> getOwnVelocityVision(int index = 0);
		shared_ptr<msl_actuator_msgs::MotionControl> getLastMotionCommand(int index = 0);
		shared_ptr<int> getCompassOrientation(int index = 0);
		shared_ptr<msl_msgs::JoystickCommand> getJoystickCommand(int index = 0);
		shared_ptr<msl_sensor_msgs::CorrectedOdometryInfo> getCorrectedOdometryInfo(int index = 0);
		shared_ptr<msl_sensor_msgs::BallHypothesisList> getBallHypothesisList(int index = 0);
		void processWorldModelData(msl_sensor_msgs::WorldModelDataPtr data);
		void processJoystickCommand(msl_msgs::JoystickCommandPtr msg);
		void processMotionBurst(msl_actuator_msgs::MotionBurstPtr msg);
		void processLightBarrier(std_msgs::BoolPtr msg);
		void processRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg);
		void processMotionControlMessage(msl_actuator_msgs::MotionControl& mc);
		void processCorrectedOdometryInfo(msl_sensor_msgs::CorrectedOdometryInfoPtr& coi);
		void processBallHypothesisList(msl_sensor_msgs::BallHypothesisListPtr& list);
		void processIMUData(msl_actuator_msgs::IMUDataPtr msg);

	private:
		RingBuffer<InformationElement<vector<double>>> distanceScan;
		RingBuffer<InformationElement<bool>> lightBarrier;
		RingBuffer<InformationElement<geometry::CNPoint2D>> opticalFlow;
		RingBuffer<InformationElement<geometry::CNPosition>> ownPositionMotion;
		RingBuffer<InformationElement<geometry::CNPosition>> ownPositionVision;
		RingBuffer<InformationElement<msl_msgs::MotionInfo>> ownVelocityMotion;
		RingBuffer<InformationElement<msl_msgs::MotionInfo>> ownVelocityVision;
		RingBuffer<InformationElement<msl_actuator_msgs::MotionControl>> lastMotionCommand;
		RingBuffer<InformationElement<int>> compass;
		RingBuffer<InformationElement<msl_msgs::JoystickCommand>> joystickCommands;
		RingBuffer<InformationElement<msl_sensor_msgs::CorrectedOdometryInfo>> ownOdometry;
		RingBuffer<InformationElement<msl_sensor_msgs::BallHypothesisList>> ballHypothesis;
		RingBuffer<InformationElement<msl_actuator_msgs::IMUData>> imuData;
		MSLWorldModel* wm;


		unsigned long maxInformationAge;
		int ownID;
		bool loggingEnabled;
	};

} /* namespace alica */

#endif /* CNC_MSL_MSL_WORLDMODEL_RAWSENSORDATA_H_ */
