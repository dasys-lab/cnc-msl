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

namespace alica
{

	DribbleCalibrationContainer::DribbleCalibrationContainer()
	{
		this->wm = msl::MSLWorldModel::get();
	}

	DribbleCalibrationContainer::~DribbleCalibrationContainer()
	{

	}

    msl_actuator_msgs::MotionControl DribbleCalibrationContainer::getBall()
	{
    	query->reset();
		msl::RobotMovement rm;
		msl_actuator_msgs::MotionControl mc;
		auto me = wm->rawSensorData->getOwnPositionVision();
		auto egoBallPos = wm->ball->getAlloBallPosition()->alloToEgo(*me);

		query->egoDestinationPoint = egoBallPos;
		query->egoAlignPoint = egoBallPos;

		mc = rm.moveToPoint(query);
		return mc;
	}

    /**
     * movement may be: FORWARD, BACKWARD, LEFT, RIGHT
     *
     * @return MotionControl to drive in a direction while using PathPlaner and avoid obstacles
     */
    msl_actuator_msgs::MotionControl DribbleCalibrationContainer::move(int movement, int translation)
    {
    	msl_actuator_msgs::MotionControl mc;

    	if (movement != FORWARD && movement != BACKWARD && movement != LEFT && movement  != RIGHT)
    	{
    		cout << "DribbleCalibrationContainer::move() -> invalid input parameter" << endl;
    		mc.senderID = -1;
    		mc.motion.translation = NAN;
    		mc.motion.rotation = NAN;
    		mc.motion.angle = NAN;
    		return mc;
    	}

    	query->reset();
    	msl::RobotMovement rm;

    	shared_ptr<geometry::CNPoint2D> egoDestination = make_shared<geometry::CNPoint2D>(0, 0) ;
    	double distance = 300;
    	// drive forward
    	egoDestination = movement == FORWARD ? make_shared<geometry::CNPoint2D>(-distance, 0) : egoDestination;
    	// drive backward
    	egoDestination = movement == BACKWARD ? make_shared<geometry::CNPoint2D>(distance, 0) : egoDestination;
    	// drive left
    	egoDestination = movement == LEFT ? make_shared<geometry::CNPoint2D>(0, distance) : egoDestination;
    	// drive right
    	egoDestination = movement == RIGHT ? make_shared<geometry::CNPoint2D>(0, -distance) : egoDestination;

    	query->egoDestinationPoint = egoDestination;
    	query->egoAlignPoint = egoDestination;

    	mc = rm.moveToPoint(query);

    	mc.motion.translation = translation;
    	return mc;
    }

	double DribbleCalibrationContainer::readConfigParameter(const char *path)
	{
		supplementary::SystemConfig* sys = supplementary::SystemConfig::getInstance();
		return (*sys)["Actuation"]->get<double>(path, NULL);
	}

	void DribbleCalibrationContainer::writeConfigParameters(vector<subsection> sections)
	{
		for (subsection section : sections)
		{

		}

	}
} /* namespace alica */
