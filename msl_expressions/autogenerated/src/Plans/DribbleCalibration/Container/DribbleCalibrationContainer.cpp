/*
 * DribbleCalibrationContainer.cpp
 *
 *  Created on: Jul 22, 2016
 *      Author: Carpe Noctem
 */

#include <Plans/DribbleCalibration/Container/DribbleCalibrationContainer.h>

#include <SystemConfig.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <RawSensorData.h>
#include <MSLFootballField.h>
#include <obstaclehandler/Obstacles.h>
#include <pathplanner/PathPlanner.h>

namespace alica
{

	DribbleCalibrationContainer::DribbleCalibrationContainer()
	{
		queueFilled = false;
		this->wm = msl::MSLWorldModel::get();
		this->query = make_shared<MovementQuery>();
		robotRadius = 0;
//		potentialAlignPoint = nullptr;
		defaultDistance = 0;
		distToObs = 0;
		changeDirections = false;
		rotateAroundTheBall = false;
		angleTolerance = 0;
		alloAlignPoint = nullptr;
		readOwnConfigParameter();
	}

	DribbleCalibrationContainer::~DribbleCalibrationContainer()
	{

	}

	msl_actuator_msgs::MotionControl DribbleCalibrationContainer::getBall()
	{
		msl::RobotMovement rm;
		query->reset();
		msl_actuator_msgs::MotionControl mc;
		shared_ptr<geometry::CNPosition> me;
		if (wm->rawSensorData->getOwnPositionVision() != nullptr)
		{
			me = wm->rawSensorData->getOwnPositionVision();
		}
		else
		{
			cerr << "DribbleCalibrationContainer::getBall() -> couldn't get vision data" << endl;
		}
		shared_ptr<geometry::CNPoint2D> egoBallPos;
		if (wm->ball->getAlloBallPosition()->alloToEgo(*me) != nullptr)
		{
			egoBallPos = wm->ball->getAlloBallPosition()->alloToEgo(*me);
		}
		else
		{
			cerr << "DribbleCalibrationContainer::getBall() -> couldn't get own position" << endl;
		}

		query->egoDestinationPoint = egoBallPos;
		query->egoAlignPoint = egoBallPos;

		mc = rm.moveToPoint(query);
		mc.motion.translation += 400;
		return mc;
	}

	/**
	 * movement may be: FORWARD, BACKWARD, LEFT, RIGHT
	 *
	 * @return MotionControl to drive in a direction while using PathPlaner and avoid obstacles
	 */
	msl_actuator_msgs::MotionControl DribbleCalibrationContainer::move(Movement movement, int translation)
	{
		msl_actuator_msgs::MotionControl mc;
		msl::RobotMovement rm;

		if (movement != Forward && movement != Backward && movement != Left && movement != Right)
		{
			cerr << "DribbleCalibrationContainer::move() -> invalid input parameter" << endl;
			return setNaN(mc);
		}

		// check if there is an obstacle
		if (!changeDirections)
			changeDirections = checkObstacles(movement, distToObs);

		if (changeDirections)
		{
			cout << "alloAlignPoint == nullptr : " << (alloAlignPoint == nullptr ? "true" : "false") << endl;
			auto me = wm->rawSensorData->getOwnPositionVision();

			if (alloAlignPoint == nullptr)
			{
				alloAlignPoint = calcNewAlignPoint(movement)->egoToAllo(*me);
			}

			shared_ptr<geometry::CNPoint2D> newAlignPoint = alloAlignPoint->alloToEgo(*me);

			cout << "newAlignPoint: " << newAlignPoint->toString() << endl;
			cout << "newALignPoint->angleTo() = " << fabs(newAlignPoint->angleTo()) << endl;

			if (fabs(newAlignPoint->angleTo()) < (M_PI - query->angleTolerance))
			{
				query->egoAlignPoint = newAlignPoint;
				query->rotateAroundTheBall = rotateAroundTheBall;
				query->angleTolerance = angleTolerance;

				cout << "aligning to new align point!" << endl;
				mc = rm.alignTo(query);
				return mc;
			}
			else if (newAlignPoint != nullptr)
			{
				cout << "reset changeDirection flag and newAlignPoint" << endl;
				alloAlignPoint = nullptr;
				changeDirections = false;
				newAlignPoint = nullptr;

				return setZero(mc);
			}
		}
		else
		{
			query->reset();

			shared_ptr<geometry::CNPoint2D> egoDestination = getEgoDestinationPoint(movement, defaultDistance);

			query->egoDestinationPoint = egoDestination;
			query->egoAlignPoint = egoDestination;

			cout << "egoDestinationPoint = " << egoDestination->toString() << endl;
//		cout << "egoAlignPoint = " << query->egoAlignPoint->toString() << endl;

			mc = rm.moveToPoint(query);

			mc.motion.translation = translation;
			mc.motion.rotation = 0;
			return mc;
		}
//		return mc;
		cout << "return mc -> NaN!" << endl;
		return setNaN(mc);
	}

	bool DribbleCalibrationContainer::checkObstacles(Movement movement, double distance)
	{
		if (movement != Forward && movement != Backward && movement != Left && movement != Right
				&& movement != ForwardRight && movement != ForwardLeft && movement != BackwardRight
				&& movement != BackwardLeft)
		{
			cerr << "DribbleCalibrationContainer::checkObstacles() -> wrong input" << endl;
			return true;
		}

		cout << "in check obstacles!" << endl;

		// currenPos 		: CNPoint2D -> ownPos
		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData->getOwnPositionVision();
		// goal				: CNPoint2D -> destPoint
		shared_ptr<geometry::CNPoint2D> egoDest = getEgoDestinationPoint(movement, distance);
		// obstacelPoint	: CNPOint2D -> obstacle
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> obs = wm->obstacles->getAlloObstaclePoints();
		// obstacleRadius	: double
//		double obsRadius;

		if (checkFieldLines(egoDest))
		{
//			cout << "field line in front of me" << endl;
			return true;
		}

		if (obs == nullptr || obs->size() == 0)
		{
			return false;
		}

//		cout << "found " << obs->size() << " obstacles" << endl;
		auto me = wm->rawSensorData->getOwnPositionVision();

		for (shared_ptr<geometry::CNPoint2D> ob : *obs)
		{
//			cout << "egoObstacle: " << ob->alloToEgo(*me)->toString() << endl;
//			cout << "egoDestination: " << egoDest->toString() << endl;
//			cout << "distance: " << distance << endl;
//			cout << "corridorCheck: " << (wm->pathPlanner->corridorCheck(ownPos->getPoint(), egoDest, ob) ? "true" : "false")<< endl;
			if (wm->pathPlanner->corridorCheck(ownPos->getPoint(), egoDest, ob->alloToEgo(*me)))
				return true;
		}

		return false;
	}

	/**
	 * @return true if the movement destination is outside the field
	 */
	bool DribbleCalibrationContainer::checkFieldLines(shared_ptr<geometry::CNPoint2D> egoDest)
	{
		// egoDestinationPoint to allo
		egoDest->x = egoDest->x == 0 ? 1 : egoDest->x;
		shared_ptr<geometry::CNPoint2D> alloDestination = egoDest->egoToAllo(
				*wm->rawSensorData->getOwnPositionVision());
		cout << "alloDestinationPoint = " << alloDestination->toString();

		// check if destinationPoint is inside the field area
		double fieldLength = wm->field->getFieldLength();
		double fieldWidth = wm->field->getFieldWidth();

		if (fabs(alloDestination->x) > (fieldLength / 2) || fabs(alloDestination->y) > (fieldWidth / 2))
		{
			cout << "Field line in front of me!" << endl;
			return true;
		}
		return false;
	}

	shared_ptr<geometry::CNPoint2D> DribbleCalibrationContainer::getEgoDestinationPoint(Movement movement,
																						double distance)
	{
		if (movement != Forward && movement != Backward && movement != Left && movement != Right
				&& movement != ForwardRight && movement != ForwardLeft && movement != BackwardRight
				&& movement != BackwardLeft)
		{
			cout << "DribbleCalibrationContainer::getEgoDestinationPoint -> wrong input!" << endl;
			return nullptr;
		}

		double beta = 45;
		double a = distance * cos(beta);
		double b = sqrt(((distance * distance) - (a * a)));

		shared_ptr<geometry::CNPoint2D> egoDestination = make_shared<geometry::CNPoint2D>(0, 0);
		// 1000 = 1m
		egoDestination = movement == Forward ? make_shared<geometry::CNPoint2D>(-distance, 0) : egoDestination;
		egoDestination = movement == Backward ? make_shared<geometry::CNPoint2D>(distance, 0) : egoDestination;
		egoDestination = movement == Left ? make_shared<geometry::CNPoint2D>(0, distance) : egoDestination;
		egoDestination = movement == Right ? make_shared<geometry::CNPoint2D>(0, -distance) : egoDestination;
		egoDestination = movement == ForwardRight ? make_shared<geometry::CNPoint2D>(-b, a) : egoDestination;
		egoDestination = movement == ForwardLeft ? make_shared<geometry::CNPoint2D>(-b, -a) : egoDestination;
		egoDestination = movement == BackwardRight ? make_shared<geometry::CNPoint2D>(b, a) : egoDestination;
		egoDestination = movement == BackwardLeft ? make_shared<geometry::CNPoint2D>(b, -a) : egoDestination;

		return egoDestination;
	}

	shared_ptr<geometry::CNPoint2D> DribbleCalibrationContainer::calcNewAlignPoint(Movement curMove)
	{
		cout << "choose new direction..." << endl;

		// use checkObstacels to choose a new direction
//		vector<Movement> movement = {Forward, Backward, Left, Right, ForwardRight, ForwardLeft, BackwardRight,
//										BackwardLeft};
		vector<Movement> movement = {Forward, ForwardRight, Right, BackwardRight, Backward, BackwardLeft, Left, ForwardLeft};

		double distance = 3000;
		shared_ptr<geometry::CNPoint2D> alignPoint;

		for (int i = 0; i < movement.size(); i++)
		{
			Movement dir = movement.at(i);

			if (!checkObstacles(dir, distance))
			{
				{
					cout << "New align point in " << movementToString[dir] << " direction..." << endl;
					cout << "curMove = " << movementToString[curMove] << endl;

					dir = curMove == Left ? getNewDirection(i, movement, 2): dir;
					dir = curMove == Right ? getNewDirection(i, movement, -2): dir;

					cout << "New specific align point in " << movementToString[dir] << " direction..." << endl;

					alignPoint = getEgoDestinationPoint(dir, distance);
					break;
				}
			}
		}

		return alignPoint;
	}

	/**
	 * helper function for calcNewAlignPoint
	 * uses vector<Movement> movement like a ring list
	 *
	 * @return the new align point if the robot is driving in another direction than forward
	 */
	DribbleCalibrationContainer::Movement DribbleCalibrationContainer::getNewDirection(int curDir, vector<Movement> movement, int next)
	{
		cout << "in getNewDirection!" << endl;
		if ((curDir - next) >= 0 && (curDir - next) < movement.size())
		{
			return movement.at(curDir - next);
		} else if ((curDir - next) > (movement.size() - 1))
		{
			int i = (curDir - next) - (movement.size() - 1);
			return movement.at(i);
		} else
		{
			int i = (movement.size() - 1) - abs(curDir - next);
			return movement.at(i);
		}
	}

	/**
	 * @return true if opQueue is filled
	 */
	bool DribbleCalibrationContainer::fillOpticalFlowQueue(int queueSize,
															shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> opQueue)
	{
		// 10s of rotating the ball
//		int queueSize = 285;

		if (wm->rawSensorData->getOpticalFlow(0) == nullptr)
		{
			cerr << "no OpticalFLow signal!" << endl;
			return false;
		}
//		cout << "queueSize = " << queueSize << endl;
//		cout << "opQueue.size() = " << opQueue->size() << endl;
		if (opQueue->size() >= queueSize)
		{
			return true;
		}
		if (!queueFilled)
		{
			cout << "filling optical flow queue!" << endl;
			queueFilled = true;
		}
		opQueue->push_back(wm->rawSensorData->getOpticalFlow(0));

		return false;
	}

	double DribbleCalibrationContainer::getAverageOpticalFlowXValue(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> queue)
	{
		return getAverageOpticalFlowValue(XValue, queue);
	}

	double DribbleCalibrationContainer::getAverageOpticalFlowYValue(shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> queue)
	{
		return getAverageOpticalFlowValue(YValue, queue);
	}

	double DribbleCalibrationContainer::getAverageOpticalFlowValue(OPValue value,
																	shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> queue)
	{
		if (value != XValue && value != YValue /*&& value != QOSValue*/)
		{
			cerr << "DribbleCalibrationContainer::getAverageOpticalFlowValue -> wrong method input!" << endl;
			return -1;
		}

		int sum = 0;
#ifdef DEBUG_DC
		cout << "in getAverageOpticalFlowValue() " << endl;
#endif
		for (shared_ptr<geometry::CNPoint2D> val : *queue)
		{
			if (value == XValue)
			{
				sum += val->x;
			}
			else
			{
//    			cout << val->y << endl;s
				sum += val->y;
			}
		}
		double ret = fabs(sum) / queue->size();
		return sum < 0 ? -ret : ret;

	}

	/**
	 * helps to read config parameters out of the Actuation.conf
	 *
	 * @path in Actuation config to the needed config parameter
	 */
	double DribbleCalibrationContainer::readConfigParameter(const char *path)
	{
		supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
		return (*sys)["Actuation"]->get<double>(path, NULL);
	}

	/**
	 * writes sections to config file
	 *
	 * @sections are the sections
	 * @path is the path to the section e.g. "ForwardDribbleSpeeds"
	 */
	void DribbleCalibrationContainer::writeConfigParameters(vector<subsection> sections, const char* path)
	{
#ifdef DEBUG_DC
		cout << "write sections to config!" << endl;
#endif
		supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
		for (subsection section : sections)
		{
			string s(path);
			s += "." + section.name;

			(*sys)["Actuation"]->set(boost::lexical_cast<std::string>(section.actuatorSpeed),
										(s + ".actuatorSpeed").c_str(), NULL);
#ifdef DEBUG_DC
			cout << "wrote: " << s.append(".actuatorSpeed").c_str() << " with value " << section.actuatorSpeed << endl;
#endif
			(*sys)["Actuation"]->set(boost::lexical_cast<std::string>(section.robotSpeed), (s + ".robotSpeed").c_str(),
			NULL);
		}
		(*sys)["Actuation"]->store();
	}

	void DribbleCalibrationContainer::readOwnConfigParameter()
	{
		supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
		robotRadius = (*sys)["Rules"]->get<double>("Rules.RobotRadius", NULL);
		defaultDistance = (*sys)["DribbleCalibration"]->get<double>("DribbleCalibration.Default.DefaultDistance", NULL);
		distToObs = (*sys)["DribbleCalibration"]->get<double>("DribbleCalibration.Default.DistanceToObstacle", NULL);
		rotateAroundTheBall = (*sys)["DribbleCalibration"]->get<bool>("DribbleCalibration.Default.RotateAroundTheBall",
		NULL);
		angleTolerance = (*sys)["DribbleCalibration"]->get<double>("DribbleCalibration.Default.AngleTolerance", NULL);
	}

	MotionControl DribbleCalibrationContainer::setZero(MotionControl mc)
	{
		mc.motion.translation = 0;
		mc.motion.angle = 0;
		mc.motion.rotation = 0;
		return mc;
	}

	MotionControl DribbleCalibrationContainer::setNaN(MotionControl mc)
	{
		mc.senderID = -1;
		mc.motion.translation = NAN;
		mc.motion.rotation = NAN;
		mc.motion.angle = NAN;
		return mc;
	}
} /* namespace alica */
