/*
 * Motion.cpp
 *
 *  Created on: Aug 19, 2015
 *      Author: Stephan Opfer
 */

#include "Motion.h"
#include <thread>
#include <signal.h>
#include <SystemConfig.h>
#include <time.h>

namespace msl_driver
{

	bool Motion::running;

	Motion::Motion(int argc, char** argv) :
			rosNode(nullptr), spinner(nullptr)
	{

		this->motionValue = new MotionSet();
		this->motionResult = new MotionSet();

		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();

		this->ownId = sc->getOwnRobotID();

		this->odometryDelay = (*sc)["Motion"]->tryGet<int>(100000, "Motion", "OdometryDelay", NULL);
		// Read slip control parameters
		this->slipControlEnabled = (*sc)["Motion"]->tryGet<bool>(false, "Motion", "SlipControl", "Enabled", NULL);
		this->slipControlMinSpeed = (*sc)["Motion"]->tryGet<double>(1250.0, "Motion", "SlipControl", "MinSpeed", NULL);
		this->slipControlDiffAngle = (*sc)["Motion"]->tryGet<double>((M_PI / 180.0) * 10.0, "Motion", "SlipControl",
																		"DiffAngle", NULL);
		this->slipControlDiffAngleMinSpeed = (*sc)["Motion"]->tryGet<double>(400.0, "Motion", "SlipControl",
																				"DiffAngleMinSpeed", NULL);
		this->slipControlOldMaxRot = (*sc)["Motion"]->tryGet<double>(M_PI / 20.0, "Motion", "SlipControl", "OldMaxRot",
		NULL);
		this->slipControlNewMinRot = (*sc)["Motion"]->tryGet<double>(M_PI / 2.0, "Motion", "SlipControl", "NewMinRot",
		NULL);

		if (this->slipControlEnabled)
		{
			std::cout << "Motion: slip control min speed            = " << this->slipControlMinSpeed << std::endl;
			std::cout << "Motion: slip control diff angle           = " << this->slipControlDiffAngle << std::endl;
			std::cout << "Motion: slip control diff angle min speed = " << this->slipControlDiffAngleMinSpeed
					<< std::endl;
			std::cout << "Motion: slip control old max rotation     = " << this->slipControlOldMaxRot << std::endl;
			std::cout << "Motion: slip control new min rotation     = " << this->slipControlNewMinRot << std::endl;
		}

		// Read acceleration compensation parameters
		this->accelCompEnabled = (*sc)["Motion"]->tryGet<bool>(false, "Motion", "AccelComp", "Enabled", NULL);
		this->accelCompMaxAccel = (*sc)["Motion"]->tryGet<double>(4000.0, "Motion", "AccelComp", "MaxAccel", NULL);
		this->accelCompMaxAngularAccel = (*sc)["Motion"]->tryGet<double>(M_PI / 2.0, "Motion", "AccelComp",
																			"MaxAngularAccel", NULL);

		if (this->accelCompEnabled)
		{
			std::cout << "Motion: max acceleration " << this->accelCompMaxAccel << std::endl;
			std::cout << "Motion: max angular acceleration " << this->accelCompMaxAngularAccel << std::endl;
		}

		// Initialize the acceleration compensation
		this->accelComp = new AccelCompensation(this->accelCompMaxAccel, this->accelCompMaxAngularAccel);

		// Set Trace-Model (according to the impera repository, we always used the CircleTrace model)
		this->traceModel = new CircleTrace();

		// Read required driver parameters
		this->driverAlivePeriod = (*sc)["Motion"]->tryGet<int>(250, "Motion", "AlivePeriod", NULL);
		this->driverOpenAttemptPeriod = (*sc)["Motion"]->tryGet<int>(1000, "Motion", "OpenAttemptPeriod", NULL);

		// Set Driver (according to the impera repository, we always used the CNMC-Driver (not the CNMCTriForce-Driver).
		// TODO: Implement class CNMC-Driver (inherits from class Driver)
		driver = new CNMC();
		this->driver->alivePeriod = this->driverAlivePeriod;
		this->driver->openAttemptPeriod = this->driverOpenAttemptPeriod;

		// Initialize the driver
		// TODO: Implement Initialize of Class Driver.
		if (!this.driver.Initialize())
		{
			this->quit = true;
			delete this->driver;
			if (ros::ok())
			{
				ros::shutdown();
			}
		}

	}

	Motion::~Motion()
	{
		delete motionValue;
		delete motionResult;
		delete motionValueOld;
		delete accelComp;
		delete traceModel;
		delete driver;
	}

	/**
	 * Initialises all ROS communcation (sub and pub).
	 */
	void Motion::initCommunication(int argc, char** argv)
	{
		ros::init(argc, argv, "MSL_TMC_Motion");
		rosNode = new ros::NodeHandle();
		spinner = new ros::AsyncSpinner(4);
		handleMotionControlSub = rosNode->subscribe("/MotionControl", 10, &Motion::handleMotionControl, (Motion*)this);
		rawOdometryInfoPub = rosNode->advertise<msl_actuator_msgs::RawOdometryInfo>("/RawOdometry", 10);
		motionStatInfoPub = rosNode->advertise<msl_actuator_msgs::MotionStatInfo>("/MotionStatInfo", 10);
		spinner->start();
	}

	void Motion::start()
	{
		if (!Motion::running)
		{
			Motion::running = true;
			this->mainThread = new thread(&Motion::run, this);
		}
	}

	/**
	 * Method for checking, whether the Motion's main thread is still running.
	 * @return running
	 */
	bool Motion::isRunning()
	{
		return running;
	}

	void Motion::notifyDriverResultAvailable(DriverData* data)
	{
		MotionResult* mr = dynamic_cast<MotionResult>(data);
		if (mr == nullptr)
		{
			return;
		}

		this.motionValueOld = mr;

		// Send the raw motion values to the vision
		msl_actuator_msgs::RawOdometryInfo ro;

		this->traceModel->trace(new double[] {mr.angle, mr.translation, mr.rotation});

		ro.position.x = this->traceModel->x;
		ro.position.y = this->traceModel->y;
		ro.position.angle = this->traceModel->angle;

		ro.motion.angle = mr.angle;
		ro.motion.translation = mr.translation;
		ro.motion.rotation = mr.rotation;

		ro.timestamp = (ulong)(DateTime.UtcNow.Ticks - this.odometryDelay);

		this->rawOdometryInfoPub.publish(ro);

		// Compute the slip control factor if requested
		if ((this->slipControlEnabled) && (this->rawMotionValuesOld != nullptr))
		{

			// Get the diff angle
			double adiff = mr.angle - this.rawMotionValuesOld[0];
			bool slip = false;

			// Normalize it
			if (adiff < -M_PI)
			{
				adiff += 2.0 * M_PI;
			}

			if (adiff > M_PI)
			{
				adiff -= 2.0 * M_PI;
			}

			if ((Math.Abs(adiff) > this.slipControlDiffAngle)
					&& (Math.Abs(this.rawMotionValuesOld[1]) > this.slipControlDiffAngleMinSpeed))
			{
				slip = true;
			}

			if ((Math.Abs(rawMotionValuesOld[2]) < this.slipControlOldMaxRot)
					&& (Math.Abs(mr.rotation) > this.slipControlNewMinRot))
			{
				slip = true;
			}

			// Compute the factor using this equation: factor = 1 / (1 + e^(-2 * factor - x)) where x \in { 0.75, 1.2 }
			if (slip)
			{
				this.slipControlFactor = 1.0 / (1.0 + Math.Exp(-(2.0 * this.slipControlFactor - 1.20)));

			}
			else
			{
				this.slipControlFactor = 1.0 / (1.0 + Math.Exp(-(3.0 * this.slipControlFactor - 0.75)));
			}

		}

		// Initialize the rawMotionValuesOld cache
		if (this.rawMotionValuesOld == null)
		{
			this.rawMotionValuesOld = new double[3];
		}

		// Copy the values
		this.rawMotionValuesOld[0] = mr.angle;
		this.rawMotionValuesOld[1] = mr.translation;
		this.rawMotionValuesOld[2] = mr.rotation;
	}

	void Motion::notifyDriverStatusChange(CNMC::StatusCode code, string message)
	{
		if (this->quit || ros::ok())
		{
			return;
		}
	}
	void Motion::handleMotionControl(msl_actuator_msgs::MotionControlPtr mc)
	{
		//	TODO: OnMotionControl
		if (this->accelCompEnabled && this->motionValueOld != nullptr)
		{
			msl_msgs::MotionInfo* m = new msl_msgs::MotionInfo();
			m->angle = this->motionValueOld->angle;
			m->translation = this->motionValueOld->translation;
			m->rotation = this->motionValueOld->rotation;
			mc->motion = *(this->accelComp->compensate(&mc->motion, m, (ulong)clock()));

		}

		std::lock_guard<std::mutex> lck(this->motionValueMutex);
		{
			// Create a new driver command
			this->motionValue->angle = mc->motion.angle;
			this->motionValue->translation = mc->motion.translation;
			this->motionValue->rotation = mc->motion.rotation;

			// Apply the slip control if enabled
			if ((this->slipControlEnabled) && (mc->motion.translation > this->slipControlMinSpeed))
			{

				this->motionValue->translation *= this->slipControlFactor;
				this->motionValue->rotation *= this->slipControlFactor;

			}
			//			TODO TEMPLATE
			//			this->driver->request<MotionSet>(this.motionValue);
		}
	}

	/**
	 * This is for handling Strg + C, although no ROS communication was running.
	 * @param sig
	 */
	void Motion::pmSigintHandler(int sig)
	{
		cout << endl << "Motion: Caught SIGINT! Terminating ..." << endl;
		running = false;

		// Call the ros signal handler method
		ros::shutdown();
	}
	/**
	 * This is for handling SIGTERM to terminate
	 * @param sig
	 */
	void Motion::pmSigTermHandler(int sig)
	{
		cout << endl << "Motion: Caught SIGTERM! Terminating ..." << endl;
		running = false;

		// Call the ros signal handler method
		ros::shutdown();
	}

} /* namespace msl_driver */

int main(int argc, char** argv)
{
	msl_driver::Motion* motion = new msl_driver::Motion(argc, argv);
	motion->initCommunication(argc, argv);
	// has to be set after Motion::initCommunication , in order to override the ROS signal handler
	signal(SIGINT, msl_driver::Motion::pmSigintHandler);
	signal(SIGTERM, msl_driver::Motion::pmSigTermHandler);
	motion->start();

	while (motion->isRunning())
	{
		chrono::milliseconds dura(500);
		this_thread::sleep_for(dura);
	}

	delete motion;
}

