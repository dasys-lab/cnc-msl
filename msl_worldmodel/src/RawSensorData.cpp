#define IMULOG false
/*

 * RawSensorData.cpp
 *
 *  Created on: Feb 18, 2015
 *      Author: Stefan Jakob
 */

#include <SystemConfig.h>
#include <container/CNPoint3D.h>
#include "Ball.h"
#include "RawSensorData.h"
#include "MSLWorldModel.h"
#include <math.h>

namespace msl
{
	std::string logFile = "/home/cn/cnws/IMU.log";
	FILE *lp= fopen(logFile.c_str(), "a");

	RawSensorData::RawSensorData(MSLWorldModel* wm, int ringbufferLength) :
			distanceScan(ringbufferLength), lightBarrier(ringbufferLength), opticalFlow(ringbufferLength), ownPositionMotion(
					ringbufferLength), ownPositionVision(ringbufferLength), ownVelocityMotion(ringbufferLength), ownVelocityVision(
					ringbufferLength), compass(ringbufferLength), joystickCommands(ringbufferLength), ownOdometry(
					ringbufferLength), lastMotionCommand(ringbufferLength), ballHypothesis(ringbufferLength), imuData(
					ringbufferLength)
	{
		this->wm = wm;
		ownID = supplementary::SystemConfig::getOwnRobotID();
		maxInformationAge = 1000000000;
		loggingEnabled = false;
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

	bool RawSensorData::getLightBarrier(int index)
	{
		auto x = lightBarrier.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return false;
		}
		return *x->getInformation();
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
	shared_ptr<msl_actuator_msgs::MotionControl> RawSensorData::getLastMotionCommand(int index)
	{
		auto x = lastMotionCommand.getLast(index);
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

	shared_ptr<int> RawSensorData::getCompassOrientation(int index)
	{
		auto x = compass.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	shared_ptr<pair<shared_ptr<geometry::CNPosition>, double>> RawSensorData::getOwnPositionVisionAndCertaincy(
			int index)
	{
		shared_ptr<pair<shared_ptr<geometry::CNPosition>, double>> ret = make_shared<
				pair<shared_ptr<geometry::CNPosition>, double>>();
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
		if (x == nullptr || wm->getTime() - x->timeStamp > 250000000)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	shared_ptr<msl_sensor_msgs::CorrectedOdometryInfo> RawSensorData::getCorrectedOdometryInfo(int index)
	{
		auto x = ownOdometry.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	shared_ptr<msl_sensor_msgs::BallHypothesisList> RawSensorData::getBallHypothesisList(int index)
	{
		auto x = ballHypothesis.getLast(index);
		if (x == nullptr || wm->getTime() - x->timeStamp > maxInformationAge)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	void RawSensorData::processRawOdometryInfo(msl_actuator_msgs::RawOdometryInfoPtr msg)
	{
		shared_ptr<InformationElement<geometry::CNPosition>> motion = make_shared<
				InformationElement<geometry::CNPosition>>(
				make_shared<geometry::CNPosition>(msg->position.x, msg->position.y, msg->position.angle),
				wm->getTime());
		ownPositionMotion.add(motion);
		shared_ptr<InformationElement<msl_msgs::MotionInfo>> vel =
				make_shared<InformationElement<msl_msgs::MotionInfo>>(make_shared<msl_msgs::MotionInfo>(msg->motion),
																		wm->getTime());
		ownVelocityMotion.add(vel);
	}

	void RawSensorData::processJoystickCommand(msl_msgs::JoystickCommandPtr msg)
	{
		if (msg->robotId == this->ownID)
		{
			/*
			 * In order to convert the boost::shared_ptr to a std::shared_ptr
			 * we use the conversion suggested in this post:
			 * http://stackoverflow.com/questions/12314967/cohabitation-of-boostshared-ptr-and-stdshared-ptr
			 */
			shared_ptr<msl_msgs::JoystickCommand> cmd = shared_ptr<msl_msgs::JoystickCommand>(
					msg.get(), [msg](msl_msgs::JoystickCommand*) mutable
					{	msg.reset();});
			shared_ptr<InformationElement<msl_msgs::JoystickCommand>> jcmd = make_shared<
					InformationElement<msl_msgs::JoystickCommand>>(cmd, wm->getTime());
			jcmd->certainty = 1.0;
			joystickCommands.add(jcmd);
		}
	}

	void RawSensorData::processMotionBurst(msl_actuator_msgs::MotionBurstPtr msg)
	{
		shared_ptr<geometry::CNPoint2D> opt = make_shared<geometry::CNPoint2D>(msg->x, msg->y);
		shared_ptr<InformationElement<geometry::CNPoint2D>> o = make_shared<InformationElement<geometry::CNPoint2D>>(
				opt, wm->getTime());
		o->certainty = msg->qos;
		opticalFlow.add(o);
	}

	void RawSensorData::processLightBarrier(std_msgs::BoolPtr msg)
	{
		shared_ptr<bool> lb = make_shared<bool>(msg->data);
		shared_ptr<InformationElement<bool>> l = make_shared<InformationElement<bool>>(lb, wm->getTime());
		l->certainty = 1.0;
		lightBarrier.add(l);
	}

	void RawSensorData::processMotionControlMessage(msl_actuator_msgs::MotionControl& cmd)
	{
		shared_ptr<msl_actuator_msgs::MotionControl> mc = make_shared<msl_actuator_msgs::MotionControl>();
		mc->motion.angle = cmd.motion.angle;
		mc->motion.translation = cmd.motion.translation;
		mc->motion.rotation = cmd.motion.rotation;
		mc->timestamp = cmd.timestamp;
		shared_ptr<InformationElement<msl_actuator_msgs::MotionControl>> smc = make_shared<
				InformationElement<msl_actuator_msgs::MotionControl>>(mc, wm->getTime());
		smc->certainty = 1;
		lastMotionCommand.add(smc);
	}

	void RawSensorData::processWorldModelData(msl_sensor_msgs::WorldModelDataPtr data)
	{
		InfoTime time = wm->getTime();

		if (data->odometry.certainty > 0)
		{
			//full odometry
			msl_sensor_msgs::CorrectedOdometryInfo test = msl_sensor_msgs::CorrectedOdometryInfo(data->odometry);
			shared_ptr<msl_sensor_msgs::CorrectedOdometryInfo> odom =
					make_shared<msl_sensor_msgs::CorrectedOdometryInfo>(data->odometry);
			shared_ptr<InformationElement<msl_sensor_msgs::CorrectedOdometryInfo>> odo = make_shared<
					InformationElement<msl_sensor_msgs::CorrectedOdometryInfo>>(odom, time);
			odo->certainty = data->odometry.certainty;
			ownOdometry.add(odo);

			//Vision
			shared_ptr<geometry::CNPosition> pos = make_shared<geometry::CNPosition>(data->odometry.position.x,
																						data->odometry.position.y,
																						data->odometry.position.angle);
			shared_ptr<InformationElement<geometry::CNPosition>> odometry = make_shared<
					InformationElement<geometry::CNPosition>>(pos, time);
			odometry->certainty = data->odometry.certainty;
			ownPositionVision.add(odometry);

			shared_ptr<msl_msgs::MotionInfo> vel = make_shared<msl_msgs::MotionInfo>(data->odometry.motion);
			shared_ptr<InformationElement<msl_msgs::MotionInfo>> v = make_shared<
					InformationElement<msl_msgs::MotionInfo>>(vel, time);
			v->certainty = data->odometry.certainty;
			ownVelocityVision.add(v);

			//Motion
			/*shared_ptr<geometry::CNPosition> posMotion = make_shared<geometry::CNPosition>(
			 data->odometry.position.x, data->odometry.position.y, data->odometry.position.angle);
			 shared_ptr<InformationElement<geometry::CNPosition>> odometryMotion = make_shared<
			 InformationElement<geometry::CNPosition>>(posMotion, time);
			 odometryMotion->certainty = data->odometry.certainty;
			 ownPositionMotion.add(odometryMotion);

			 // TODO: this is the same motion as for vision motion !?
			 shared_ptr<msl_msgs::MotionInfo> velMotion = make_shared<msl_msgs::MotionInfo>(data->odometry.motion);
			 shared_ptr<InformationElement<msl_msgs::MotionInfo>> vMotion = make_shared<
			 InformationElement<msl_msgs::MotionInfo>>(velMotion, time);
			 vMotion->certainty = data->odometry.certainty;
			 ownVelocityMotion.add(vMotion);*/
		}

		shared_ptr<geometry::CNPoint3D> ballPos = make_shared<geometry::CNPoint3D>(data->ball.point.x,
																					data->ball.point.y,
																					data->ball.point.z);
		shared_ptr<geometry::CNPoint3D> ballVel = make_shared<geometry::CNPoint3D>(data->ball.velocity.vx,
																					data->ball.velocity.vy,
																					data->ball.velocity.vz);

		//cout << "RawSensorData: Ball X:" << ballVel->x << ", Y:" << ballVel->y << endl;
		if (data->ball.confidence < 0.00000001)
			this->wm->ball->updateBallPos(nullptr, nullptr, data->ball.confidence);
		else
			this->wm->ball->updateBallPos(ballPos, ballVel, data->ball.confidence);

		shared_ptr<vector<double>> dist = make_shared<vector<double>>(data->distanceScan.sectors);

		// TODO This is a Taker workaround, should be removed when real error was found
		int count = 0;
		while (dist.use_count() == 0)
		{
			if (count > 5)
				return;
			dist = make_shared<vector<double>>(data->distanceScan.sectors);
			++count;
		}

		shared_ptr<InformationElement<vector<double>>> distance = make_shared<InformationElement<vector<double>>>(dist, time);

		// TODO This is a Taker workaround, should be removed when real error was found
		count = 0;
		while (dist.use_count() == 1)
		{
			if (count > 5)
				return;
			dist = make_shared<vector<double>>(data->distanceScan.sectors);
			++count;
		}
		distance->certainty = data->ball.confidence;
		distanceScan.add(distance);
		wm->getVisionDataEventTrigger()->run();
	}

	void RawSensorData::processCorrectedOdometryInfo(msl_sensor_msgs::CorrectedOdometryInfoPtr& coi)
	{
		shared_ptr<geometry::CNPosition> opt = make_shared<geometry::CNPosition>(coi->position.x, coi->position.y,
																					coi->position.angle);
		shared_ptr<InformationElement<geometry::CNPosition>> o = make_shared<InformationElement<geometry::CNPosition>>(
				opt, wm->getTime());
		o->certainty = coi->position.certainty;
		ownPositionVision.add(o);
		this->wm->ball->updateOnLocalizationData(coi->imageTime);
	}

	void RawSensorData::processBallHypothesisList(msl_sensor_msgs::BallHypothesisListPtr& list)
	{
		shared_ptr<msl_sensor_msgs::BallHypothesisList> nList = make_shared<msl_sensor_msgs::BallHypothesisList>(*list);
		shared_ptr<InformationElement<msl_sensor_msgs::BallHypothesisList>> o = make_shared<
				InformationElement<msl_sensor_msgs::BallHypothesisList>>(nList, wm->getTime());
		o->certainty = 1;
		ballHypothesis.add(o);
		this->wm->ball->updateOnBallHypothesisList(list->imageTime);
	}

	void log(int index, float value)
	{
		fprintf(lp, "%d:%f\n", index, value);
		fflush(lp);

	}

	void RawSensorData::processIMUData(msl_actuator_msgs::IMUDataPtr msg)
	{
		shared_ptr<msl_actuator_msgs::IMUData> cmd = make_shared<msl_actuator_msgs::IMUData>();
		cmd->accelSens = msg->accelSens;
		cmd->acceleration = msg->acceleration;
		cmd->gyro = msg->gyro;
		cmd->gyroSens = msg->gyroSens;
		cmd->magnet = msg->magnet;
		cmd->magnetSens = msg->magnetSens;
		cmd->temperature = msg->temperature;
		cmd->time = msg->time;
		shared_ptr<InformationElement<msl_actuator_msgs::IMUData>> o = make_shared<InformationElement<msl_actuator_msgs::IMUData>>(cmd, wm->getTime());
		o->certainty = 1;
		imuData.add(o);

		// TODO IMU-Baustelle Kai/Marci
		double bearing = atan2(cmd->magnet.y, cmd->magnet.x);
		if(IMULOG)
		{
			log(0, atan2(cmd->magnet.y, cmd->magnet.x));
			//log(1, imuData.getAverageMod());
		}
	}
} /* namespace alica */

