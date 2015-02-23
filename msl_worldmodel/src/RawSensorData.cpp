/*
 * RawSensorData.cpp
 *
 *  Created on: Feb 18, 2015
 *      Author: Stefan Jakob
 */

#include "RawSensorData.h"
#include "MSLWorldModel.h"

namespace msl
{

	RawSensorData::RawSensorData(MSLWorldModel* wm, int ringbufferLength) :
			distanceScan(ringbufferLength), ballPosition(ringbufferLength), ballVelocity(ringbufferLength), lightBarrier(
					ringbufferLength), opticalFlow(ringbufferLength), ownPositionMotion(ringbufferLength), ownPositionVision(
					ringbufferLength), ownVelocityMotion(ringbufferLength), ownVelocityVision(ringbufferLength), compass(
					ringbufferLength)
	{
		this->wm = wm;
	}

	RawSensorData::~RawSensorData()
	{
	}

	shared_ptr<vector<double> > RawSensorData::getDistanceScan(int index)
	{
		auto x = distanceScan.getLast(index);
		if(x==nullptr) return nullptr;
		return x->getInformation();
	}

	shared_ptr<bool> RawSensorData::getLightBarrier(int index)
	{
		auto x = lightBarrier.getLast(index);
		if(x==nullptr) return nullptr;
		return x->getInformation();
	}

	shared_ptr<CNPoint2D> RawSensorData::getOpticalFlow(int index)
	{
		auto x = opticalFlow.getLast(index);
		if(x==nullptr) return nullptr;
		return x->getInformation();
	}

	shared_ptr<CNPosition> RawSensorData::getOwnPositionMotion(int index)
	{
		auto x = ownPositionMotion.getLast(index);
		if(x==nullptr) return nullptr;
		return x->getInformation();
	}

	shared_ptr<CNPosition> RawSensorData::getOwnPositionVision(int index)
	{
		auto x = ownPositionVision.getLast(index);
		if(x==nullptr) return nullptr;
		return x->getInformation();
	}

	shared_ptr<msl_msgs::MotionInfo> RawSensorData::getOwnVelocityMotion(int index)
	{
		auto x = ownVelocityMotion.getLast(index);
		if(x==nullptr) return nullptr;
		return x->getInformation();
	}

	shared_ptr<msl_msgs::MotionInfo> RawSensorData::getOwnVelocityVision(int index)
	{
		auto x = ownVelocityVision.getLast(index);
		if(x==nullptr) return nullptr;
		return x->getInformation();
	}

	shared_ptr<CNPoint2D> RawSensorData::getBallPosition(int index)
	{
		auto x = ballPosition.getLast(index);
		if(x==nullptr) return nullptr;
		return x->getInformation();
	}

	shared_ptr<CNVelocity2D> RawSensorData::getBallVelocity(int index)
	{
		auto x = ballVelocity.getLast(index);
		if(x==nullptr) return nullptr;
		return x->getInformation();
	}

	shared_ptr<int> RawSensorData::getCompass(int index)
	{
		auto x = compass.getLast(index);
		if(x==nullptr) return nullptr;
		return x->getInformation();
	}

	shared_ptr<pair<shared_ptr<CNPoint2D>, double>> RawSensorData::getBallPositionAndCertaincy(int index)
	{
		shared_ptr<pair<shared_ptr<CNPoint2D>, double>> ret = make_shared<pair<shared_ptr<CNPoint2D>, double>>();
		auto x = ballPosition.getLast(index);
		if(x==nullptr) return nullptr;
		ret->first = x->getInformation();
		ret->second = x->certainty;
		return ret;
	}

	shared_ptr<pair<shared_ptr<CNPosition>, double>> RawSensorData::getOwnPositionVisionAndCertaincy(int index)
	{
		shared_ptr<pair<shared_ptr<CNPosition>, double>> ret = make_shared<pair<shared_ptr<CNPosition>, double>>();
		auto x = ownPositionVision.getLast(index);
		if(x==nullptr) return nullptr;
		ret->first = x->getInformation();
		ret->second = x->certainty;
		return ret;
	}

	void RawSensorData::processWorldModelData(msl_sensor_msgs::WorldModelDataPtr data)
	{
		unsigned long time = wm->getTime();



		if (data->odometry.certainty > 0)
		{
			shared_ptr<CNPosition> pos = make_shared<CNPosition>(data->odometry.position.x, data->odometry.position.y,
																	data->odometry.position.angle);
			shared_ptr<InformationElement<CNPosition>> odometry = make_shared<InformationElement<CNPosition>>(pos,
																												time);
			odometry->certainty = data->odometry.certainty;
			ownPositionVision.add(odometry);
			shared_ptr<msl_msgs::MotionInfo> vel = make_shared<msl_msgs::MotionInfo>(data->odometry.motion);
			shared_ptr<InformationElement<msl_msgs::MotionInfo>> v = make_shared<
					InformationElement<msl_msgs::MotionInfo>>(vel, time);
			v->certainty = data->odometry.certainty;
			ownVelocityVision.add(v);
		}

		if (data->ball.confidence > 0)
		{
			shared_ptr<CNPoint2D> ballPos = make_shared<CNPoint2D>(data->ball.point.x, data->ball.point.y);
			shared_ptr<InformationElement<CNPoint2D>> ball = make_shared<InformationElement<CNPoint2D>>(ballPos, time);
			ball->certainty = data->ball.confidence;
			ballPosition.add(ball);

			shared_ptr<CNVelocity2D> ballVel = make_shared<CNVelocity2D>(data->ball.velocity.vx,
																			data->ball.velocity.vy);
			shared_ptr<InformationElement<CNVelocity2D>> ballV = make_shared<InformationElement<CNVelocity2D>>(ballVel,
																												time);
			ballV->certainty = data->ball.confidence;
			ballVelocity.add(ballV);
		}

		shared_ptr<vector<double>> dist = make_shared<vector<double>>(data->distanceScan.sectors);
		shared_ptr<InformationElement<vector<double>>> distance = make_shared<InformationElement<vector<double>>>(dist,
				time);
		distance->certainty = data->ball.confidence;
		distanceScan.add(distance);
	}

} /* namespace alica */
