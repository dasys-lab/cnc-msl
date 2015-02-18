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
#include "msl_actuator_msgs/MotionBurst.h"
#include "container/CNPosition.h"
#include "container/CNVelocity2D.h"
#include "container/CNPoint2D.h"
#include "msl_sensor_msgs/WorldModelData.h"


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
		shared_ptr<CNPoint2D> getBallPosition(int index = 0);
		shared_ptr<CNVelocity2D> getBallVelocity(int index = 0);
		shared_ptr<bool> getLightBarrier(int index = 0);
		shared_ptr<CNPoint2D> getOpticalFlow(int index = 0);
		shared_ptr<CNPosition> getOwnPositionMotion(int index = 0);
		shared_ptr<CNPosition> getOwnPositionVision(int index = 0);
		shared_ptr<msl_msgs::MotionInfo> getOwnVelocityMotion(int index = 0);
		shared_ptr<msl_msgs::MotionInfo> getOwnVelocityVision(int index = 0);
		shared_ptr<int> getCompass(int index = 0);
		void processWorldModelData(msl_sensor_msgs::WorldModelDataPtr data);

	private:
		RingBuffer<InformationElement<vector<double>>> distanceScan;
		RingBuffer<InformationElement<CNPoint2D>> ballPosition;
		RingBuffer<InformationElement<CNVelocity2D>> ballVelocity;
		RingBuffer<InformationElement<bool>> lightBarrier;
		RingBuffer<InformationElement<CNPoint2D>> opticalFlow;
		RingBuffer<InformationElement<CNPosition>> ownPositionMotion;
		RingBuffer<InformationElement<CNPosition>> ownPositionVision;
		RingBuffer<InformationElement<msl_msgs::MotionInfo>> ownVelocityMotion;
		RingBuffer<InformationElement<msl_msgs::MotionInfo>> ownVelocityVision;
		RingBuffer<InformationElement<int>> compass;
		MSLWorldModel* wm;

	};

} /* namespace alica */

#endif /* CNC_MSL_MSL_WORLDMODEL_RAWSENSORDATA_H_ */
