/*
 * RawSensorData.h
 *
 *  Created on: Feb 18, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_RAWSENSORDATA_H_
#define CNC_MSL_MSL_WORLDMODEL_RAWSENSORDATA_H_

#include "RingBuffer.h"
#include "InformationElement.h"
#include <vector>
#include "msl_sensor_msgs/BallInfo.h"
#include "msl_msgs/MotionInfo.h"
#include "msl_msgs/JoystickCommand.h"
#include "msl_actuator_msgs/MotionBurst.h"
#include "container/CNPosition.h"
#include "container/CNVelocity2D.h"
#include "container/CNPoint2D.h"
#include "msl_sensor_msgs/WorldModelData.h"
#include "msl_actuator_msgs/RawOdometryInfo.h"


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
		shared_ptr<geometry::CNPoint2D> getBallPosition(int index = 0);
		shared_ptr<pair<shared_ptr<geometry::CNPoint2D>, double>> getBallPositionAndCertaincy(int index = 0);
		shared_ptr<geometry::CNVelocity2D> getBallVelocity(int index = 0);
		shared_ptr<bool> getLightBarrier(int index = 0);
		shared_ptr<geometry::CNPoint2D> getOpticalFlow(int index = 0);
		double getOpticalFlowQoS(int index = 0);
		shared_ptr<geometry::CNPosition> getOwnPositionMotion(int index = 0);
		shared_ptr<geometry::CNPosition> getOwnPositionVision(int index = 0);
		shared_ptr<pair<shared_ptr<geometry::CNPosition>, double>> getOwnPositionVisionAndCertaincy(int index = 0);
		shared_ptr<msl_msgs::MotionInfo> getOwnVelocityMotion(int index = 0);
		shared_ptr<msl_msgs::MotionInfo> getOwnVelocityVision(int index = 0);
		shared_ptr<int> getCompassOrientation(int index = 0);
		shared_ptr<msl_msgs::JoystickCommand> getJoystickCommand(int index = 0);
		void processWorldModelData(msl_sensor_msgs::WorldModelDataPtr data);
		void processJoystickCommand(msl_msgs::JoystickCommandPtr msg);
		void processMotionBurst(msl_actuator_msgs::MotionBurstPtr msg);
		void processRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg);

	private:
		RingBuffer<InformationElement<vector<double>>> distanceScan;
		RingBuffer<InformationElement<geometry::CNPoint2D>> ballPosition;
		RingBuffer<InformationElement<geometry::CNVelocity2D>> ballVelocity;
		RingBuffer<InformationElement<bool>> lightBarrier;
		RingBuffer<InformationElement<geometry::CNPoint2D>> opticalFlow;
		RingBuffer<InformationElement<geometry::CNPosition>> ownPositionMotion;
		RingBuffer<InformationElement<geometry::CNPosition>> ownPositionVision;
		RingBuffer<InformationElement<msl_msgs::MotionInfo>> ownVelocityMotion;
		RingBuffer<InformationElement<msl_msgs::MotionInfo>> ownVelocityVision;
		RingBuffer<InformationElement<int>> compass;
		RingBuffer<InformationElement<msl_msgs::JoystickCommand>> joystickCommands;
		MSLWorldModel* wm;

		unsigned long maxInformationAge;
		int ownID;

	};

} /* namespace alica */

#endif /* CNC_MSL_MSL_WORLDMODEL_RAWSENSORDATA_H_ */
