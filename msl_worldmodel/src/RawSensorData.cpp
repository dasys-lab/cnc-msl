/*
 * RawSensorData.cpp
 *
 *  Created on: Feb 18, 2015
 *      Author: Stefan Jakob
 */

#include <SystemConfig.h>
#include "RawSensorData.h"
#include "MSLWorldModel.h"

namespace msl
{

	RawSensorData::RawSensorData(MSLWorldModel* wm, int ringbufferLength) :
			distanceScan(ringbufferLength), ballPosition(ringbufferLength), ballVelocity(ringbufferLength), lightBarrier(ringbufferLength), opticalFlow(
					ringbufferLength), ownPositionMotion(ringbufferLength), ownPositionVision(ringbufferLength), ownVelocityMotion(ringbufferLength), ownVelocityVision(
					ringbufferLength), compass(ringbufferLength), joystickCommands(ringbufferLength)
	{
		this->wm = wm;
		ownID = supplementary::SystemConfig::getOwnRobotID();
		maxInformationAge = 1000000000;
	}

	RawSensorData::~RawSensorData()
	{
	}

	shared_ptr<vector<double> > RawSensorData::getDistanceScan(int index)
	{
		auto x = distanceScan.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	shared_ptr<bool> RawSensorData::getLightBarrier(int index)
	{
		auto x = lightBarrier.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	shared_ptr<geometry::CNPoint2D> RawSensorData::getOpticalFlow(int index)
	{
		auto x = opticalFlow.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	double RawSensorData::getOpticalFlowQoS(int index)
	{
		auto x = opticalFlow.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return 0;
		}
		return x->certainty;
	}

	shared_ptr<geometry::CNPosition> RawSensorData::getOwnPositionMotion(int index)
	{
		auto x = ownPositionMotion.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	shared_ptr<geometry::CNPosition> RawSensorData::getOwnPositionVision(int index)
	{
		auto x = ownPositionVision.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	shared_ptr<msl_msgs::MotionInfo> RawSensorData::getOwnVelocityMotion(int index)
	{
		auto x = ownVelocityMotion.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	shared_ptr<msl_msgs::MotionInfo> RawSensorData::getOwnVelocityVision(int index)
	{
		auto x = ownVelocityVision.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	shared_ptr<geometry::CNPoint2D> RawSensorData::getBallPosition(int index)
	{
		auto x = ballPosition.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	shared_ptr<geometry::CNVelocity2D> RawSensorData::getBallVelocity(int index)
	{
		auto x = ballVelocity.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	shared_ptr<int> RawSensorData::getCompassOrientation(int index)
	{
		auto x = compass.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	shared_ptr<pair<shared_ptr<geometry::CNPoint2D>, double>> RawSensorData::getBallPositionAndCertaincy(int index)
	{
		shared_ptr<pair<shared_ptr<geometry::CNPoint2D>, double>> ret = make_shared<pair<shared_ptr<geometry::CNPoint2D>, double>>();
		auto x = ballPosition.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		ret->first = x->getInformation();
		ret->second = x->certainty;
		return ret;
	}

	shared_ptr<pair<shared_ptr<geometry::CNPosition>, double>> RawSensorData::getOwnPositionVisionAndCertaincy(int index)
	{
		shared_ptr<pair<shared_ptr<geometry::CNPosition>, double>> ret = make_shared<pair<shared_ptr<geometry::CNPosition>, double>>();
		auto x = ownPositionVision.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		ret->first = x->getInformation();
		ret->second = x->certainty;
		return ret;
	}

	shared_ptr<msl_msgs::JoystickCommand> RawSensorData::getJoystickCommand(int index)
	{
		auto x = joystickCommands.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	void RawSensorData::processRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg)
	{
		shared_ptr<InformationElement<geometry::CNPosition>> motion = make_shared<InformationElement<geometry::CNPosition>>(
				make_shared<geometry::CNPosition>(msg->position.x, msg->position.y, msg->position.angle), wm->getTime());
		ownPositionMotion.add(motion);
		shared_ptr<InformationElement<msl_msgs::MotionInfo>> vel = make_shared<InformationElement<msl_msgs::MotionInfo>>(
				make_shared<msl_msgs::MotionInfo>(msg->motion), wm->getTime());
		ownVelocityMotion.add(vel);
	}

	void RawSensorData::processJoystickCommand(msl_msgs::JoystickCommandPtr msg)
	{
		if (msg->robotId == this->ownID)
		{
			shared_ptr<msl_msgs::JoystickCommand> cmd = make_shared<msl_msgs::JoystickCommand>(*msg);
			shared_ptr<InformationElement<msl_msgs::JoystickCommand>> jcmd = make_shared<InformationElement<msl_msgs::JoystickCommand>>(
					cmd, wm->getTime());
			jcmd->certainty = 1.0;
			joystickCommands.add(jcmd);
		}
	}

	void RawSensorData::processMotionBurst(msl_actuator_msgs::MotionBurstPtr msg)
	{
		shared_ptr<geometry::CNPoint2D> opt = make_shared<geometry::CNPoint2D>(msg->x, msg->y);
		shared_ptr<InformationElement<geometry::CNPoint2D>> o = make_shared<InformationElement<geometry::CNPoint2D>>(opt, wm->getTime());
		o->certainty = msg->qos;
		opticalFlow.add(o);
	}

	void RawSensorData::processWorldModelData(msl_sensor_msgs::WorldModelDataPtr data)
	{
		InfoTime time = wm->getTime();

		if (data->odometry.certainty > 0)
		{
			//Vision
			shared_ptr<geometry::CNPosition> pos = make_shared<geometry::CNPosition>(data->odometry.position.x, data->odometry.position.y,
																						data->odometry.position.angle);
			shared_ptr<InformationElement<geometry::CNPosition>> odometry = make_shared<InformationElement<geometry::CNPosition>>(pos, time);
			odometry->certainty = data->odometry.certainty;
			ownPositionVision.add(odometry);

			shared_ptr<msl_msgs::MotionInfo> vel = make_shared<msl_msgs::MotionInfo>(data->odometry.motion);
			shared_ptr<InformationElement<msl_msgs::MotionInfo>> v = make_shared<InformationElement<msl_msgs::MotionInfo>>(vel, time);
			v->certainty = data->odometry.certainty;
			ownVelocityVision.add(v);

			//Motion
			shared_ptr<geometry::CNPosition> posMotion = make_shared<geometry::CNPosition>(data->odometry.position.x, data->odometry.position.y,
																							data->odometry.position.angle);
			shared_ptr<InformationElement<geometry::CNPosition>> odometryMotion = make_shared<InformationElement<geometry::CNPosition>>(posMotion,
																																		time);
			odometryMotion->certainty = data->odometry.certainty;
			ownPositionMotion.add(odometryMotion);

			shared_ptr<msl_msgs::MotionInfo> velMotion = make_shared<msl_msgs::MotionInfo>(data->odometry.motion);
			shared_ptr<InformationElement<msl_msgs::MotionInfo>> vMotion = make_shared<InformationElement<msl_msgs::MotionInfo>>(velMotion, time);
			vMotion->certainty = data->odometry.certainty;
			ownVelocityMotion.add(vMotion);
		}

		if (data->ball.confidence > 0)
		{
			shared_ptr<geometry::CNPoint2D> ballPos = make_shared<geometry::CNPoint2D>(data->ball.point.x, data->ball.point.y);
			shared_ptr<InformationElement<geometry::CNPoint2D>> ball = make_shared<InformationElement<geometry::CNPoint2D>>(ballPos, time);
			ball->certainty = data->ball.confidence;
			ballPosition.add(ball);

			shared_ptr<geometry::CNVelocity2D> ballVel = make_shared<geometry::CNVelocity2D>(data->ball.velocity.vx, data->ball.velocity.vy);
			shared_ptr<InformationElement<geometry::CNVelocity2D>> ballV = make_shared<InformationElement<geometry::CNVelocity2D>>(ballVel, time);
			ballV->certainty = data->ball.confidence;
			ballVelocity.add(ballV);
		}

		shared_ptr<vector<double>> dist = make_shared<vector<double>>(data->distanceScan.sectors);
		shared_ptr<InformationElement<vector<double>>> distance = make_shared<InformationElement<vector<double>>>(dist,
				time);
		distance->certainty = data->ball.confidence;
		distanceScan.add(distance);

		this->wm->ball.updateOnWorldModelData();
	}

} /* namespace alica */

