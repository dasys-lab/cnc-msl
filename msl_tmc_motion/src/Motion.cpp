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

namespace msl_driver
{

	bool Motion::running;

	Motion::Motion(int argc, char** argv)
	{
		this->motionValue = new MotionSet();
		this->motionResult = new MotionSet();

		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		std::string driversPath = (*sc)["Motion"]->get<string>("Motion.DriversPath", NULL);
		std::string driverName = (*sc)["Motion"]->get<string>("Motion.Driver", NULL);
		std::string modelsPath = (*sc)["Motion"]->get<string>("Motion.ModelsPath", NULL);
		std::string traceModelName = (*sc)["Motion"]->get<string>("Motion.TraceModel", NULL);

		this->ownId = sc->getOwnRobotID();

		this->odometryDelay = (*sc)["Motion"]->tryGet<int>(100000,"Motion","OdometryDelay", NULL);
		// Read slip control parameters
		this->slipControlEnabled = (*sc)["Motion"]->tryGet<bool>(false, "Motion", "SlipControl", "Enabled", NULL);
		this->slipControlMinSpeed = (*sc)["Motion"]->tryGet<double>(1250.0, "Motion", "SlipControl", "MinSpeed", NULL);
		this->slipControlDiffAngle = (*sc)["Motion"]->tryGet<double>((M_PI / 180.0) * 10.0, "Motion", "SlipControl", "DiffAngle", NULL);
		this->slipControlDiffAngleMinSpeed = (*sc)["Motion"]->tryGet<double>(400.0, "Motion", "SlipControl", "DiffAngleMinSpeed", NULL);
		this->slipControlOldMaxRot = (*sc)["Motion"]->tryGet<double>(M_PI / 20.0, "Motion", "SlipControl", "OldMaxRot", NULL);
		this->slipControlNewMinRot = (*sc)["Motion"]->tryGet<double>(M_PI / 2.0, "Motion", "SlipControl", "NewMinRot", NULL);


		/*




		 if (this.slipControlEnabled) {
		 Trace.WriteLine("Motion: slip control min speed            = {0}", this.slipControlMinSpeed);
		 Trace.WriteLine("Motion: slip control diff angle           = {0}", this.slipControlDiffAngle);
		 Trace.WriteLine("Motion: slip control diff angle min speed = {0}", this.slipControlDiffAngleMinSpeed);
		 Trace.WriteLine("Motion: slip control old max rotation     = {0}", this.slipControlOldMaxRot);
		 Trace.WriteLine("Motion: slip control new min rotation     = {0}", this.slipControlNewMinRot);
		 }

		 // Read acceleration compensation parameters
		 this.accelCompEnabled = this.sc["Motion"].TryGetBool(false, "Motion", "AccelComp", "Enabled");
		 this.accelCompMaxAccel = this.sc["Motion"].TryGetDouble(4000.0, "Motion", "AccelComp", "MaxAccel");
		 this.accelCompMaxAngularAccel = this.sc["Motion"].TryGetDouble(Math.PI / 2.0, "Motion", "AccelComp", "MaxAngularAccel");

		 //			Trace.WriteLine("Motion: acceleration compensation {0}", (this.accelCompEnabled ? "enabled" : "disabled"));

		 if (this.accelCompEnabled) {
		 Trace.WriteLine("Motion: max acceleration         = {0}", this.accelCompMaxAccel);
		 Trace.WriteLine("Motion: max angular acceleration = {0}", this.accelCompMaxAngularAccel);
		 }

		 // Initialize the acceleration compensation
		 this.accelComp = new AccelCompensation(this.accelCompMaxAccel, this.accelCompMaxAngularAccel);

		 // Load the trace model
		 try {
		 this.traceModel = TraceModel.Load(this.sc.LibPath + modelsPath, traceModelName);

		 } catch (Exception e) {
		 Debug.WriteLine(e.InnerException.Message);
		 Debug.WriteLine("Motion: error loading trace model {0} from {1}!", driverName, driversPath);
		 return;
		 }

		 // Read required driver parameters
		 this.driverAlivePeriod = this.sc["Motion"].TryGetInt(250, "Motion", "AlivePeriod");
		 this.driverOpenAttemptPeriod = this.sc["Motion"].TryGetInt(1000, "Motion", "OpenAttemptPeriod");

		 //			Trace.WriteLine("Motion: driver alive period        = {0}", this.driverAlivePeriod);
		 //			Trace.WriteLine("Motion: driver open attempt period = {0}", this.driverOpenAttemptPeriod);

		 this.RawOdomPub = new Publisher(node, "RawOdometry", RawOdometryInfo.TypeId, 1);
		 this.MotionStatPub = new Publisher(node,"MotionStatInfo",MotionStatInfo.TypeId,1);
		 this.sendRosCompliant = this.sc["Motion"].TryGetBool(true,"Motion","SendRosCompliant");

		 if(this.sendRosCompliant) {
		 this.rmc = new RosMsgConnector();
		 }

		 // Load the controller driver
		 try {
		 this.driver = Driver.Load(this.sc.LibPath + driversPath, driverName);
		 this.driver.AlivePeriod = this.driverAlivePeriod;
		 this.driver.OpenAttemptPeriod = this.driverOpenAttemptPeriod;
		 this.driver.StatusChange += OnDriverStatusChange;
		 this.driver.ResultAvailable += OnDriverResultAvailable;

		 } catch (Exception e) {
		 if(e.InnerException!=null) Debug.WriteLine(e.InnerException.Message);
		 Debug.WriteLine("Motion: error loading motor driver {0} from {1}!", driverName, driversPath);
		 return;
		 }




		 // Initialize the driver
		 if (!this.driver.Initialize()) {
		 Close();
		 }
		 node.Subscribe("MotionControl",this.OnMotionControl);
		 //node.Subscribe("cmd_vel",this.OnTwist);


		 this.monitoringTimer = new System.Threading.Timer(MonitoringCallback, null, 1000, 1000); */

	}

	void Motion::handleMotionControl(msl_actuator_msgs::MotionStatInfoPtr msi)
	{
		// todo: this was formally OnMotionControl in the C# code
	}

	Motion::~Motion()
	{
		// TODO Auto-generated destructor stub
	}

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

	void Motion::run()
	{
		// todo
	}

	/**
	 * Method for checking, whether the Motion's main thread is still running.
	 * @return running
	 */
	bool Motion::isRunning()
	{
		return running;
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

