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

    	if (movement != DRIBBLE_FORWARD && movement != DRIBBLE_BACKWARD && movement != DRIBBLE_LEFT && movement  != DRIBBLE_RIGHT)
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
    	egoDestination = movement == DRIBBLE_FORWARD ? make_shared<geometry::CNPoint2D>(-distance, 0) : egoDestination;
    	// drive backward
    	egoDestination = movement == DRIBBLE_BACKWARD ? make_shared<geometry::CNPoint2D>(distance, 0) : egoDestination;
    	// drive left
    	egoDestination = movement == DRIBBLE_LEFT ? make_shared<geometry::CNPoint2D>(0, distance) : egoDestination;
    	// drive right
    	egoDestination = movement == DRIBBLE_RIGHT ? make_shared<geometry::CNPoint2D>(0, -distance) : egoDestination;

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

			(*sys)["Actuation"]->set(boost::lexical_cast < std::string > (section.actuatorSpeed), (s + ".actuatorSpeed").c_str() , NULL);
#ifdef DEBUG_DC
			cout << "wrote: " << s.append(".actuatorSpeed").c_str() << " with value " << section.actuatorSpeed << endl;
#endif
			(*sys)["Actuation"]->set(boost::lexical_cast < std::string > (section.robotSpeed), (s + ".robotSpeed").c_str() , NULL);
		}
		(*sys)["Actuation"]->store();
	}
} /* namespace alica */
