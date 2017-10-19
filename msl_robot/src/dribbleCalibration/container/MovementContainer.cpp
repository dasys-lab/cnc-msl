/*
 * MovementContainer.cpp
 *
 *  Created on: Dec 14, 2016
 *      Author: cn
 */

#include <msl_robot/dribbleCalibration/container/MovementContainer.h>
#include <SystemConfig.h>
#include <Ball.h>
#include <MSLWorldModel.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <RawSensorData.h>
#include <MSLFootballField.h>
#include <obstaclehandler/Obstacles.h>
#include <pathplanner/PathPlanner.h>

namespace msl
{
	MovementContainer::MovementContainer()
	{
		distToObs = 0;
		rotateAroundTheBall = false;
		angleTolerance = 0;
		changeDirections = 0;
		wm = msl::MSLWorldModel::get();
		defaultDistance = 0;
		changeDirFlag = false;
		query = make_shared<msl::MovementQuery>();
		readOwnConfigParameter();
	}

	MovementContainer::~MovementContainer()
	{
		// TODO Auto-generated destructor stub
	}

	MotionControl MovementContainer::getBall()
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
			cerr << "MovementContainer::getBall() -> couldn't get vision data" << endl;
		}
		shared_ptr<geometry::CNPoint2D> egoBallPos;
		if (wm->ball->getAlloBallPosition()->alloToEgo(*me) != nullptr)
		{
			egoBallPos = wm->ball->getAlloBallPosition()->alloToEgo(*me);
		}
		else
		{
			cerr << "MovementContainer::getBall() -> couldn't get own position" << endl;
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
	msl_actuator_msgs::MotionControl MovementContainer::move(Movement movement, int translation)
	{
		msl_actuator_msgs::MotionControl mc;
		msl::RobotMovement rm;

		if (movement != Forward && movement != Backward && movement != Left && movement != Right)
		{
			cerr << "MovementContainer::move() -> invalid input parameter" << endl;
			return setNaN(mc);
		}

		// for output
		if (changeDirFlag)
		{
			cout << "changeDirections... " << endl;
			changeDirFlag = false;
		}

		// check if there is an obstacle
		if (!changeDirections && checkObstacles(movement, defaultDistance))
		{
			changeDirections = true;
			changeDirFlag = true;
		}

		// if there is an obstacle or we are in front of a field line, change directions
		if (changeDirections)
		{
			auto me = wm->rawSensorData->getOwnPositionVision();

			if (alloAlignPoint == nullptr)
			{
				alloAlignPoint = calcNewAlignPoint(movement)->egoToAllo(*me);
			}

			shared_ptr<geometry::CNPoint2D> newAlignPoint = alloAlignPoint->alloToEgo(*me);

#ifdef DEBUG_MOVE_CONT
			cout << "MC::move() newAlignPoint: " << newAlignPoint->toString() << endl;
			cout << "MC::move() newALignPoint->angleTo() = " << fabs(newAlignPoint->angleTo()) << endl;
#endif

			if (fabs(newAlignPoint->angleTo()) < (M_PI - query->angleTolerance))
			{

#ifdef DEBUG_MOVE_CONT
				cout << "rotateAroundTheBall = " << (rotateAroundTheBall ? "true" : "false") << endl;
				cout << "angleTolerance = " << angleTolerance << endl;
#endif
				query->egoAlignPoint = newAlignPoint;
				query->rotateAroundTheBall = rotateAroundTheBall;
				query->angleTolerance = angleTolerance;

				cout << "query->egoAlignPoint = " << query->egoAlignPoint->toString();
				mc = rm.alignTo(query);

#ifdef DEBUG_MOVE_CONT
				cout << "MC::move() changeDirections: " << (changeDirections ? "true" : "false")
				<< " -> rotating to new align point" << endl;
				cout << "MC::move() MotionControl translation = " << mc.motion.translation << endl;
				cout << "MC::move() MotionControl rotation = " << mc.motion.rotation << endl;
				cout << "MC::move() MotionControl angle = " << mc.motion.angle << endl;
#endif

				return mc;
			}
			else if (newAlignPoint != nullptr)
			{
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
#ifdef DEBUG_MOVE_CONT
			cout << "MC::move() egoDestinationPoint = " << egoDestination->toString();
#endif
			mc = rm.moveToPoint(query);
			mc.motion.translation = translation;
#ifdef DEBUG_MOVE_CONT
			cout << "MC::move() changeDirections: " << (changeDirections ? "true" : "false") << " drive normally" << endl;
#endif
			return mc;
		}
		cerr << "MovementContainer::move() MotionControl mc = NaN!" << endl;
		return setNaN(mc);
	}

	/**
	 * @movement describes the direction in witch you will check for obstacles
	 *
	 * @return true if there is an obstacle in movement direction
	 */
	bool MovementContainer::checkObstacles(Movement movement, double distance)
	{
		if (movement != Forward && movement != Backward && movement != Left && movement != Right
				&& movement != ForwardRight && movement != ForwardLeft && movement != BackwardRight
				&& movement != BackwardLeft)
		{
			cerr << "MovementContainer::checkObstacles() -> wrong input" << endl;
			return true;
		}

		// currenPos 		: CNPoint2D -> ownPos
		shared_ptr<geometry::CNPosition> ownPos = wm->rawSensorData->getOwnPositionVision();
		// goal				: CNPoint2D -> destPoint
		shared_ptr<geometry::CNPoint2D> egoDest = getEgoDestinationPoint(movement, distance);
		// obstacelPoint	: CNPOint2D -> obstacle
		shared_ptr<vector<shared_ptr<geometry::CNPoint2D>>> obs = wm->obstacles->getAlloObstaclePoints();

		if (checkFieldLines(egoDest))
		{
			// field line in front of me
			return true;
		}

		if (obs == nullptr || obs->size() == 0)
		{
			return false;
		}

		auto me = wm->rawSensorData->getOwnPositionVision();

		for (shared_ptr<geometry::CNPoint2D> ob : *obs)
		{
			return wm->pathPlanner->corridorCheck(ownPos->getPoint(), egoDest, ob->alloToEgo(*me));
		}

		return false;
	}

	shared_ptr<geometry::CNPoint2D> MovementContainer::calcNewAlignPoint(Movement curMove)
	{
#ifdef DEBUG_MOVE_CONT
		cout << "MC::calcNewAlignPoint choose new direction..." << endl;
#endif

		// use checkObstacels to choose a new direction
		vector<Movement> movement = {Forward, ForwardRight, Right, BackwardRight, Backward, BackwardLeft, Left,
										ForwardLeft};

		double distance = 3000;
		shared_ptr<geometry::CNPoint2D> alignPoint;

		for (int i = 0; i < movement.size(); i++)
		{
			Movement dir = movement.at(i);

			if (!checkObstacles(dir, distance))
			{
				{
#ifdef DEBUG_MOVE_CONT
					cout << "MC::calcNewAlignPoint curMove = " << movementToString[curMove] << endl;
					cout << "MC::calcNewAlignPoint New align point in " << movementToString[dir] << " direction..." << endl;
#endif
					dir = curMove == Left ? getNewDirection(i, movement, 4) : dir;
					dir = curMove == Right ? getNewDirection(i, movement, -4) : dir;
#ifdef DEBUG_MOVE_CONT
					cout << "MC::calcNewAlignPoint New specific align point in " << movementToString[dir] << " direction..."
					<< endl;
#endif
					alignPoint = getEgoDestinationPoint(dir, distance);
					break;
				}
			}
		}
		return alignPoint;
	}

	/**
	 * @movement describes the movement direction
	 * @distance to the returned ego destination point
	 *
	 * @return an ego destination point depending on the movement direction
	 */
	shared_ptr<geometry::CNPoint2D> MovementContainer::getEgoDestinationPoint(Movement movement, double distance)
	{
		if (movement != Forward && movement != Backward && movement != Left && movement != Right
				&& movement != ForwardRight && movement != ForwardLeft && movement != BackwardRight
				&& movement != BackwardLeft)
		{
			cout << "MovementContainer::getEgoDestinationPoint -> wrong input!" << endl;
			return nullptr;
		}

		double beta = 45;
		double a = distance * cos(beta);
		double b = sqrt(((distance * distance) - (a * a)));

		shared_ptr<geometry::CNPoint2D> egoDestination = make_shared<geometry::CNPoint2D>(0, 0);
		// 1000 = 1m

		switch (movement)
		{
			case Forward:
				egoDestination = make_shared<geometry::CNPoint2D>(-distance, 0);
				break;
			case Backward:
				egoDestination = make_shared<geometry::CNPoint2D>(distance, 0);
				break;
			case Left:
				egoDestination = make_shared<geometry::CNPoint2D>(0, -distance);
				break;
			case Right:
				egoDestination = make_shared<geometry::CNPoint2D>(0, distance);
				break;
			case ForwardRight:
				egoDestination = make_shared<geometry::CNPoint2D>(-b, a);
				break;
			case ForwardLeft:
				egoDestination = make_shared<geometry::CNPoint2D>(-b, -a);
				break;
			case BackwardRight:
				egoDestination = make_shared<geometry::CNPoint2D>(b, a);
				break;
			case BackwardLeft:
				egoDestination = make_shared<geometry::CNPoint2D>(b, -a);
				break;
			default:
				cerr << "MovementContainer::getEgoDestination() wrong movement parameter value!" << endl;
				break;
		}

		return egoDestination;
	}

	/**
	 * @return true if the movement destination is outside the field
	 */
	bool MovementContainer::checkFieldLines(shared_ptr<geometry::CNPoint2D> egoDest)
	{
#ifdef DEBUG_MOVE_CONT
		cout << "MC::checkFieldLines egoDestination = " << egoDest->toString();
#endif

		auto me = wm->rawSensorData->getOwnPositionVision();

		// egoDestinationPoint to allo
		shared_ptr<geometry::CNPoint2D> alloDestination = egoDest->egoToAllo(*me);

#ifdef DEBUG_MOVE_CONT
		cout << "MC::checkFieldLines alloDestinationPoint = " << alloDestination->toString();
#endif

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

	/**
	 * helper function for calcNewAlignPoint
	 * uses vector<Movement> movement like a ring list
	 *
	 * @return the new align point if the robot is driving in another direction than forward
	 */
	MovementContainer::Movement MovementContainer::getNewDirection(int curDir, vector<Movement> movement, int next)
	{
#ifdef DEBUG_MOVE_CONT
		cout << "MC::getNewDirection: curDir = " << curDir << " next = " << next << " movement.size = "
		<< (int)movement.size() << endl;
#endif

		if ((curDir - next) >= 0 && (curDir - next) < (int)movement.size())
		{
#ifdef DEBUG_MOVE_CONT
			cout
			<< "MC::getNewDirection: ((curDir - next) >= 0 && (curDir - next) < (int) movement.size()) -> return movement.at(curDir - next)"
			<< endl;
#endif
			return movement.at(curDir - next);

		}
		else if ((curDir - next) > ((int)movement.size() - 1))
		{
			int i = (curDir - next) - ((int)movement.size() - 1);
#ifdef DEBUG_MOVE_CONT
			cout << "MC::getNewDirection: ((curDir - next) > ((int) movement.size() - 1)) i = " << i << endl;
#endif
			return movement.at(i);

		}
		else
		{
			int i = ((int)movement.size() - 1) - abs(curDir - next);
#ifdef DEBUG_MOVE_CONT
			cout << "MC::getNewDirection: i = " << i << " abs(curDir - next) = abs(" << curDir << " - " << next << ") = "
			<< abs(curDir - next) << endl;
#endif
			return movement.at(i);
		}
	}

	MotionControl MovementContainer::setZero(MotionControl mc)
	{
		mc.motion.translation = 0;
		mc.motion.angle = 0;
		mc.motion.rotation = 0;
		return mc;
	}

	MotionControl MovementContainer::setNaN(MotionControl mc)
	{
		mc.motion.translation = NAN;
		mc.motion.rotation = NAN;
		mc.motion.angle = NAN;
		return mc;
	}

	void MovementContainer::readOwnConfigParameter()
	{
		supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
//	robotRadius = (*sys)["Rules"]->get<double>("Rules.RobotRadius", NULL);
		defaultDistance = (*sys)["DribbleCalibration"]->get<double>("DribbleCalibration.Default.DefaultDistance", NULL);
		rotateAroundTheBall = (*sys)["DribbleCalibration"]->get<bool>("DribbleCalibration.Default.RotateAroundTheBall",
		NULL);
		angleTolerance = (*sys)["DribbleCalibration"]->get<double>("DribbleCalibration.Default.AngleTolerance", NULL);
	}
}
