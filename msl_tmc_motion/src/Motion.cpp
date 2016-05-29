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
#include <Configuration.h>
#include <time.h>

namespace msl_driver {

bool Motion::running = false;

Motion::Motion(int argc, char** argv) :
		rosNode(nullptr), spinner(nullptr) {
	this->my_serial = nullptr;
	this->motionValue = nullptr;

	this->sc = supplementary::SystemConfig::getInstance();

	this->ownId = this->sc->getOwnRobotID();

	this->odometryDelay = (*sc)["Motion"]->tryGet<int>(100000, "Motion",
			"OdometryDelay", NULL);
	// Read slip control parameters
	this->slipControlEnabled = (*sc)["Motion"]->tryGet<bool>(false, "Motion",
			"SlipControl", "Enabled", NULL);
	this->slipControlMinSpeed = (*sc)["Motion"]->tryGet<double>(1250.0,
			"Motion", "SlipControl", "MinSpeed", NULL);
	this->slipControlDiffAngle = (*sc)["Motion"]->tryGet<double>(
			(M_PI / 180.0) * 10.0, "Motion", "SlipControl", "DiffAngle", NULL);
	this->slipControlDiffAngleMinSpeed = (*sc)["Motion"]->tryGet<double>(400.0,
			"Motion", "SlipControl", "DiffAngleMinSpeed", NULL);
	this->slipControlOldMaxRot = (*sc)["Motion"]->tryGet<double>(M_PI / 20.0,
			"Motion", "SlipControl", "OldMaxRot",
			NULL);
	this->slipControlNewMinRot = (*sc)["Motion"]->tryGet<double>(M_PI / 2.0,
			"Motion", "SlipControl", "NewMinRot",
			NULL);

	if (this->slipControlEnabled) {
		std::cout << "Motion: slip control min speed            = "
				<< this->slipControlMinSpeed << std::endl;
		std::cout << "Motion: slip control diff angle           = "
				<< this->slipControlDiffAngle << std::endl;
		std::cout << "Motion: slip control diff angle min speed = "
				<< this->slipControlDiffAngleMinSpeed << std::endl;
		std::cout << "Motion: slip control old max rotation     = "
				<< this->slipControlOldMaxRot << std::endl;
		std::cout << "Motion: slip control new min rotation     = "
				<< this->slipControlNewMinRot << std::endl;
	}

	this->isLogging = 0;

	// Read required driver parameters
	this->driverAlivePeriod = (*sc)["Motion"]->tryGet<int>(250, "Motion",
			"AlivePeriod", NULL);
	this->driverOpenAttemptPeriod = (*sc)["Motion"]->tryGet<int>(1000, "Motion",
			"OpenAttemptPeriod", NULL);
}

Motion::~Motion() {
	if (this->motionValue != nullptr)
		delete motionValue;
}

/**
 * Initialises all ROS communcation (sub and pub).
 */
void Motion::initCommunication(int argc, char** argv) {
	ros::init(argc, argv, "MSL_TMC_Motion");
	rosNode = new ros::NodeHandle();
	spinner = new ros::AsyncSpinner(4);
	handleMotionControlSub = rosNode->subscribe("/MotionControl", 10,
			&Motion::handleMotionControl, (Motion*) this);
	rawOdometryInfoPub = rosNode->advertise<msl_actuator_msgs::RawOdometryInfo>(
			"/RawOdometry", 10);
	motionStatInfoPub = rosNode->advertise<msl_actuator_msgs::MotionStatInfo>(
			"/MotionStatInfo", 10);
	spinner->start();
}

void Motion::initialize() {
	this->minCycleTime = (*sc)["Motion"]->tryGet<long>(1, "Motion", "CNMC",
			"MinCycleTime", NULL);

	this->device = (*sc)["Motion"]->get<string>("Motion", "CNMC", "Device",
	NULL);

	this->initReadTimeout = (*sc)["Motion"]->tryGet<int>(1500, "Motion", "CNMC",
			"InitReadTimeout", NULL);
	this->readTimeout = (*sc)["Motion"]->tryGet<int>(250, "Motion", "CNMC",
			"ReadTimeout", NULL);
	this->writeTimeout = (*sc)["Motion"]->tryGet<int>(250, "Motion", "CNMC",
			"WriteTimeout", NULL);

	this->radius = (*sc)["Motion"]->get<double>("Motion", "CNMC", "RobotRadius",
	NULL);
	if (this->radius < 10) {
		std::cerr << "ROBOT RADIUS TOO LOW!!!" << std::endl;
	}

	this->maxVelocity = (*sc)["Motion"]->get<double>("Motion", "CNMC",
			"MaxVelocity", NULL);
	if (this->maxVelocity < 1) {
		std::cerr << "MAX VELOCITY TOO LOW!!" << std::endl;
	}

	this->logOdometry = (*sc)["Motion"]->get<bool>("Motion", "CNMC",
			"LogOdometry", NULL);

	// copied that from Mops Motion.conf!
	this->logTypes = make_shared<vector<string>>(initializer_list<string> {
			"ERRORINT", "MGOAL", "MOTION" });
	this->logTypesAvailable = make_shared<vector<string>>(
			initializer_list<string> { "RPM", "PWM", "RPMGOAL", "CURRENT",
					"MGOAL", "MREQUEST", "MAXPWM", "ERRORINT", "MOTION",
					"MSMOOTH", "RPMSMOOTH" });

	getMotorConfig();
}

void Motion::logging_goalie_init() {

	supplementary::Configuration *motion = (*this->sc)["Motion"];
	isLogging = motion->get<bool>("Motion", "Logging", "LogStuff", NULL);
	if (isLogging) {
		logFile = motion->get<std::string>("Motion", "Logging", "LogFile",
		NULL);
		lp = fopen(logFile.c_str(), "a");
		if (lp == NULL) {
			printf("Cannot Open Log File!\n");
			exit(-1);
		}
	}
}

void Motion::log_goalie() {

	if (isLogging) {
		fprintf(lp, "%f\t%f\t%f\t%f\t%f\t%f\t\n", rawOdoInfo.position.angle,
				rawOdoInfo.position.x, rawOdoInfo.position.y,
				rawOdoInfo.motion.angle, rawOdoInfo.motion.rotation,
				rawOdoInfo.motion.translation);
	}
}

bool Motion::open() {
// TODO: make baudrate a parameter
	this->my_serial = new serial::Serial(this->device, 57600,
			serial::Timeout::simpleTimeout(this->initReadTimeout));

	cout << "Is the serial port open?";
	if (!my_serial->isOpen()) {
		cerr << "TMC-Motion: Unable to open serial port : " << this->device
				<< " Errno: " << strerror(errno) << endl;
		return false;
	}

	this->my_serial->setTimeout(serial::Timeout::max(), this->readTimeout, 0,
			this->writeTimeout, 0);

//### SEND MOTOR CONFIG
	this->sendMotorConfig();

//### READY
	this->controllerIsActive = true;

	return true;
}

void Motion::sendData(shared_ptr<CNMCPacket> packet) {
	auto bytes = packet->getBytes();
//		cout << "TMC-Motion: Sending: ";
//		for (uint8_t byte : (*bytes))
//		{
//			cout << hex << static_cast<int>(byte) << " ";
//		}
//		cout << endl << dec << endl;
	size_t numBytesWritten = this->my_serial->write((*bytes).data(),
			bytes->size());

//		cout << "TMC-Motion: sendData - numBytesWritten: " << numBytesWritten << endl;

}

unique_ptr<CNMCPacket> Motion::readData() {
	uint8_t b[1];
	bool quoted = false;
	vector<uint8_t> data;

	do {
		if (this->my_serial->read(b, 1) == 0)
			return move(unique_ptr<CNMCPacket>());
	} while (b[0] != CNMCPacket::START_HEADER);

	data.push_back(b[0]);

	while (true) {
		if (this->my_serial->read(b, 1) == 0)
			return move(unique_ptr<CNMCPacket>());

		if (b[0] == CNMCPacket::QUOTE && !quoted) {
			quoted = true;
			continue;
		}

		//do not add end header to data
		if (b[0] == CNMCPacket::END_HEADER && !quoted) {
			break;
		}

		quoted = false;
		data.push_back(b[0]);
	}

	return CNMCPacket::getInstance(data.data(), data.size());
}

void Motion::checkSuccess(shared_ptr<CNMCPacket> cmd) {
	auto result = readData();
	if (!cmd->isExpectedResponse(move(result))) {
		// TODO implement toString of CNMCPacket
		//cerr << "Error setting CNMC " << cmd.toString() << "\n Response was " << result.toString() << endl;
	}
}

void Motion::sendMotorConfig() {
	shared_ptr<CNMCPacketConfigure> configPacket;

//gear ratio
	configPacket = make_shared<CNMCPacketConfigure>();
	shared_ptr<vector<uint8_t>> values = make_shared<vector<uint8_t>>();
	values->push_back(1);
	values->push_back((uint8_t) this->mc.gearReduction);
	configPacket->setData(CNMCPacket::ConfigureCmd::GearRatio, values);
	this->sendData(configPacket);
	this->checkSuccess(configPacket);

//encoder ticks per (half) rotation
	configPacket = make_shared<CNMCPacketConfigure>();
	configPacket->setData(CNMCPacket::ConfigureCmd::EncoderTicksPerRot,
			(short) this->mc.resolution);
	this->sendData(configPacket);
	this->checkSuccess(configPacket);

//wheel Radius
	configPacket = make_shared<CNMCPacketConfigure>();
	configPacket->setData(CNMCPacket::ConfigureCmd::WheelRadius,
			(short) (this->mc.wheelRadius * 10));
	this->sendData(configPacket);
	this->checkSuccess(configPacket);

//Robot Radius
	configPacket = make_shared<CNMCPacketConfigure>();
	configPacket->setData(CNMCPacket::ConfigureCmd::RobotRadius,
			(short) this->radius);
	this->sendData(configPacket);
	this->checkSuccess(configPacket);

//maxRPM
	int result = (int) (this->mc.maxSpeed / this->mc.gearReduction);
	configPacket = make_shared<CNMCPacketConfigure>();
	configPacket->setData(CNMCPacket::ConfigureCmd::MaxRPM, (short) result);
	this->sendData(configPacket);
	this->checkSuccess(configPacket);

	shared_ptr<CNMCPacketCtrlConfigure> ctrlPacket;
	short tmp;

//PIDKp
	ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
	tmp = (short) std::round(8192 * fmin(3, fmax(-3, this->mc.pidKd)));
	ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::PIDKp, tmp);
	this->sendData(ctrlPacket);
	this->checkSuccess(ctrlPacket);

//PIDKb
	ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
	tmp = (short) std::round(8192 * fmin(3, fmax(-3, this->mc.pidB)));
	ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::PIDb, tmp);
	this->sendData(ctrlPacket);
	this->checkSuccess(ctrlPacket);

//PIDKi
	ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
	tmp = (short) std::round(8192 * fmin(3, fmax(-3, this->mc.pidKi)));
	ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::PIDKi, tmp);
	this->sendData(ctrlPacket);
	this->checkSuccess(ctrlPacket);

//PIDKd
	ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
	tmp = (short) std::round(8192 * fmin(3, fmax(-3, this->mc.pidKd)));
	ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::PIDKd, tmp);
	this->sendData(ctrlPacket);
	this->checkSuccess(ctrlPacket);

//PIDKdi
	ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
	tmp = (short) std::round(8192 * fmin(3, fmax(-3, this->mc.pidKdi)));
	ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::PIDKdi, tmp);
	this->sendData(ctrlPacket);
	this->checkSuccess(ctrlPacket);

//linFactor
	ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
	tmp = (short) std::round(8192 * fmin(3, fmax(-3, this->mc.linFactor)));
	ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::LinearFactor, tmp);
	this->sendData(ctrlPacket);
	this->checkSuccess(ctrlPacket);

//smoothFactor
	ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
	tmp = (short) std::round(8192 * fmin(3, fmax(-3, this->mc.smoothFactor)));
	ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::SmoothFactor, tmp);
	this->sendData(ctrlPacket);
	this->checkSuccess(ctrlPacket);

//maxErrorInt
	ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
	ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::MaxErrorInt,
			(short) this->mc.maxErrorInt);
	this->sendData(ctrlPacket);
	this->checkSuccess(ctrlPacket);

//Rotation Error Weight
	ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
	tmp = (short) std::round(
			8192 * fmin(3, fmax(-3, this->mc.rotationErrorWeight)));
	ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::RotationErrorW, tmp);
	this->sendData(ctrlPacket);
	this->checkSuccess(ctrlPacket);

	ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
	tmp = (short) std::round(
			8192 * fmin(3, fmax(-3, this->mc.rotationErrorByVeloWeight)));
	ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::RotationErrorVeloW, tmp);
	this->sendData(ctrlPacket);
	this->checkSuccess(ctrlPacket);

	ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
	tmp = (short) std::round(
			8192 * fmin(3, fmax(-3, this->mc.rotationErrorByAccelWeight)));
	ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::RotationErrorAccelW, tmp);
	this->sendData(ctrlPacket);
	this->checkSuccess(ctrlPacket);

//DeadBand
	ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
	ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::DeadBand,
			(short) this->mc.deadBand);
	this->sendData(ctrlPacket);
	this->checkSuccess(ctrlPacket);

//Lower Accel Bound
	ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
	ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::LowerAccelBound,
			(short) this->mc.accelBoundMin);
	this->sendData(ctrlPacket);
	this->checkSuccess(ctrlPacket);

//Higher Accel Bound
	ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
	ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::HigherAccelBound,
			(short) this->mc.accelBoundMax);
	this->sendData(ctrlPacket);
	this->checkSuccess(ctrlPacket);

//Max Rotation Acceleration
	ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
	tmp = (short) std::round(64 * this->mc.rotationAccelBound);
	ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::MaxRotationAccel, tmp);
	this->sendData(ctrlPacket);
	this->checkSuccess(ctrlPacket);

//Fail Safe
	ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
	shared_ptr<vector<short>> vals = make_shared<vector<short>>();
	vals->push_back((short) this->mc.failSafeRPMBound);
	vals->push_back((short) this->mc.failSafePWMBound);
	vals->push_back((short) this->mc.failSafeCycles);
	ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::FailSafeValues, vals);
	this->sendData(ctrlPacket);
	this->checkSuccess(ctrlPacket);

//Current Control:
	if (this->mc.controlCurrent) {
		ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
		ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::CurrentErrorBound,
				(short) this->mc.currentErrorBound);
		this->sendData(ctrlPacket);
		this->checkSuccess(ctrlPacket);

		ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
		ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::CurrentKp,
				(short) this->mc.currentKp);
		this->sendData(ctrlPacket);
		this->checkSuccess(ctrlPacket);

		ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
		ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::CurrentKi,
				(short) this->mc.currentKi);
		this->sendData(ctrlPacket);
		this->checkSuccess(ctrlPacket);

		ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
		ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::CurrentKd,
				(short) this->mc.currentKd);
		this->sendData(ctrlPacket);
		this->checkSuccess(ctrlPacket);
	}

	int8_t logmode = 0x00;
	for (int i = 0; i < this->logTypesAvailable->size(); i++) {
		auto iter = std::find(this->logTypes->begin(), this->logTypes->end(),
				this->logTypesAvailable->at(i));
		if (iter == this->logTypes->end()) {
			logmode = -1;
		} else {
			logmode = (int8_t) std::distance(this->logTypes->begin(), iter);
		}

		configPacket = make_shared<CNMCPacketConfigure>();
		configPacket->setData(CNMCPacket::ConfigureCmd::SetLogMode,
				make_shared<vector<int8_t>>((int8_t) i, logmode));
		this->sendData(configPacket);
		this->checkSuccess(configPacket);
	}

//cycle time
	configPacket = make_shared<CNMCPacketConfigure>();
	configPacket->setData(CNMCPacket::ConfigureCmd::CycleTime, (short) 5);
	this->sendData(configPacket);
	this->checkSuccess(configPacket);

//COMMIT
	ctrlPacket = make_shared<CNMCPacketCtrlConfigure>();
	ctrlPacket->setData(CNMCPacket::CtrlConfigureCmd::Commit);
	this->sendData(ctrlPacket);
	this->checkSuccess(ctrlPacket);

//cycle time
	configPacket = make_shared<CNMCPacketConfigure>();
	configPacket->setData(CNMCPacket::ConfigureCmd::Mode, (short) 1);
	this->sendData(configPacket);
	this->checkSuccess(configPacket);

//Toggle Logging
	configPacket = make_shared<CNMCPacketConfigure>();
	shared_ptr<vector<uint8_t>> valByte = make_shared<vector<uint8_t>>();
	valByte->push_back(this->logOdometry ? (uint8_t) 1 : (uint8_t) 0);
	configPacket->setData(CNMCPacket::ConfigureCmd::ToggleOdoLog, valByte);
	this->sendData(configPacket);
	this->checkSuccess(configPacket);
}

void Motion::getMotorConfig() {
	this->mc.resolution = (*sc)["Motion"]->get<short>("Motion", "CNMC",
			"Motors", "EncoderResolution", NULL);
	this->mc.maxSpeed = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Motors",
			"MaxMotorSpeed", NULL);
	this->mc.maxCurrent = (*sc)["Motion"]->get<short>("Motion", "CNMC",
			"Motors", "MaxCurrent", NULL);
	this->mc.limitedCurrent = (*sc)["Motion"]->get<short>("Motion", "CNMC",
			"Motors", "LimitedCurrent", NULL);
	this->mc.wheelRadius = (*sc)["Motion"]->get<short>("Motion", "CNMC",
			"Motors", "WheelRadius", NULL);
	this->mc.gearReduction = (*sc)["Motion"]->get<short>("Motion", "CNMC",
			"Motors", "GearReduction", NULL);

//Controller values
	this->mc.pidKp = (*sc)["Motion"]->get<double>("Motion", "CNMC",
			"Controller", "PIDKp", NULL);
	this->mc.pidB = (*sc)["Motion"]->get<double>("Motion", "CNMC", "Controller",
			"PIDB", NULL);
	this->mc.pidKi = (*sc)["Motion"]->get<double>("Motion", "CNMC",
			"Controller", "PIDKi", NULL);
	this->mc.pidKd = (*sc)["Motion"]->get<double>("Motion", "CNMC",
			"Controller", "PIDKd", NULL);
	this->mc.pidKdi = (*sc)["Motion"]->get<double>("Motion", "CNMC",
			"Controller", "PIDKdi", NULL);
	this->mc.linFactor = (*sc)["Motion"]->tryGet<double>(0.0, "Motion", "CNMC",
			"Controller", "LinearFactor",
			NULL);
	this->mc.smoothFactor = (*sc)["Motion"]->tryGet<double>(1.0, "Motion",
			"CNMC", "Controller", "SmoothFactor",
			NULL);

	this->mc.maxErrorInt = (*sc)["Motion"]->tryGet<short>(1000, "Motion",
			"CNMC", "Controller", "MaxErrorInt",
			NULL);

	this->mc.rotationErrorWeight = (*sc)["Motion"]->get<double>("Motion",
			"CNMC", "Controller", "RotationErrorWeight", NULL);
	this->mc.rotationErrorByVeloWeight = (*sc)["Motion"]->get<double>("Motion",
			"CNMC", "Controller", "RotationErrorByVeloWeight", NULL);
	this->mc.rotationErrorByAccelWeight = (*sc)["Motion"]->get<double>("Motion",
			"CNMC", "Controller", "RotationErrorByAccelWeight", NULL);

	this->mc.deadBand = (*sc)["Motion"]->get<short>("Motion", "CNMC",
			"Controller", "DeadBand", NULL);

	this->mc.failSafeRPMBound = (*sc)["Motion"]->get<short>("Motion", "CNMC",
			"Controller", "FailSafeRPMBound",
			NULL);
	this->mc.failSafePWMBound = (*sc)["Motion"]->get<short>("Motion", "CNMC",
			"Controller", "FailSafePWMBound",
			NULL);
	this->mc.failSafeCycles = (*sc)["Motion"]->get<short>("Motion", "CNMC",
			"Controller", "FailSafeCycles", NULL);

	this->mc.accelBoundMin = (*sc)["Motion"]->tryGet<double>(0.0, "Motion",
			"CNMC", "Controller", "MinimalAccelerationAllowed");

	this->mc.accelBoundMax = (*sc)["Motion"]->tryGet<double>(0.0, "Motion",
			"CNMC", "Controller", "MaximalAccelerationAllowed");

	this->mc.rotationAccelBound = (*sc)["Motion"]->tryGet<double>(0.0, "Motion",
			"CNMC", "Controller", "MaxRotationAccel");

	this->mc.currentErrorBound = (*sc)["Motion"]->tryGet<short>(0, "Motion",
			"CNMC", "Controller", "CurrentErrorBound", NULL);
	if (this->mc.currentErrorBound == 0) {
		this->mc.controlCurrent = false;
	} else {
		this->mc.controlCurrent = true;
		this->mc.currentKp = (*sc)["Motion"]->get<double>("Motion", "CNMC",
				"Controller", "CurrentKp", NULL);
		this->mc.currentKi = (*sc)["Motion"]->get<double>("Motion", "CNMC",
				"Controller", "CurrentKi", NULL);
		this->mc.currentKd = (*sc)["Motion"]->get<double>("Motion", "CNMC",
				"Controller", "CurrentKd", NULL);
	}

	this->mc.denominator = (*sc)["Motion"]->get<short>("Motion", "CNMC",
			"Motors", "ThermalMotorConstantDenominator", NULL);
	this->mc.numerator = (*sc)["Motion"]->get<short>("Motion", "CNMC", "Motors",
			"ThermalMotorConstantNumerator",
			NULL);

	int delta = (int) (((2.0 * M_PI * (double) mc.wheelRadius)
			/ (((double) mc.resolution) * (double) mc.gearReduction)) * 1000.0);

	this->mc.vmax = (delta * mc.resolution * 2.0 * mc.maxSpeed)
			/ (60.0 * 1000.0);
}

void Motion::start() {
	if (!Motion::running) {
		Motion::running = true;
		this->runThread = std::thread(&Motion::run, this);
	}
}

/**
 * Method for checking, whether the Motion's main thread is still running.
 * @return running
 */
bool Motion::isRunning() {
	return Motion::running;
}

void Motion::calcOdoPosition() {

//calc position and angle since Motion has been initialised, initial: (0,0,0)
	int newX, newY;
	double newAngle, trans, transX, transY, rot;
	double rawOdoAngle;

//determine old position
	double lastAngle = lastOdoInfo.position.angle;
	shared_ptr<geometry::CNPoint2D> lastPos = make_shared<geometry::CNPoint2D>(
			lastOdoInfo.position.x, lastOdoInfo.position.y);

// determine time driven
	ros::Time currTime = ros::Time::now();
	uint64_t currNanoSeconds = (currTime.sec * 1000000000UL + currTime.nsec);

	double timeSinceLastOdo = (double) (currNanoSeconds - lastOdoInfo.timestamp)
			/ 1000000000.0;

//determine new position from last odo info
	rawOdoAngle = lastOdoInfo.motion.angle;
	trans = lastOdoInfo.motion.translation;
	rot = lastOdoInfo.motion.rotation;
	transX = cos(rawOdoAngle) * trans;
	transY = sin(rawOdoAngle) * trans;
	shared_ptr<geometry::CNPoint2D> translation = make_shared<
			geometry::CNPoint2D>(transX, transY);
	shared_ptr<geometry::CNPoint2D> middle;
	shared_ptr<geometry::CNPoint2D> newPos;

//angle:

	newAngle = lastAngle + rot * timeSinceLastOdo;

	if (newAngle > M_PI) {
		newAngle -= 2 * M_PI;
	} else if (newAngle < -M_PI) {
		newAngle += 2 * M_PI;
	}

//position: calc distance driven on a circular path

	double angleDriven = rot * timeSinceLastOdo;

//there is a rotational velocity, so we are driving on a circular path
	if (rot != 0) {

		double radiusLength = abs(trans / rot);

		shared_ptr<geometry::CNPoint2D> radiusVect;

		if (rot < 0) {
			radiusVect = translation->normalize()->rotate(M_PI/2) * radiusLength;
		} else {
			radiusVect = translation->normalize()->rotate(-M_PI/2) * radiusLength;
		}

		middle = lastPos - radiusVect;

		newPos = middle + radiusVect->rotate(angleDriven);
	} else {
		newPos = lastPos + translation * timeSinceLastOdo;
	}

	newX = newPos->x;
	newY = newPos->y;

//update position
	rawOdoInfo.position.x = newX;
	rawOdoInfo.position.y = newY;
	rawOdoInfo.position.angle = newAngle;

	lastOdoInfo = rawOdoInfo;

}

void Motion::run() {
	MotionSet* requestOld = nullptr;
	MotionSet* request = nullptr;

	chrono::steady_clock::time_point lastCommandTimestamp =
			std::chrono::steady_clock::now();

// Loop until the driver is closed
	while (Motion::running) {
		// 1 Tick = 100ns, 10 Ticks = 1us
		// remember the time, processing was last triggered
		this->cycleLastTimestamp = std::chrono::steady_clock::now();

		// Get the next request from the queue
		{
			std::lock_guard<std::mutex> lck(this->motionValueMutex);

			request = this->motionValue;
			this->motionValue = nullptr;
		}

		if (request == nullptr) {
			// TODO make time configurable, currently a request is send 250 ms
			if (requestOld != nullptr
					&& chrono::duration_cast<chrono::milliseconds>(
							std::chrono::steady_clock::now()
									- lastCommandTimestamp).count() > 250) {
				requestOld = nullptr;
			}

			if (requestOld != nullptr && Motion::running) {
				this->executeRequest(requestOld);
			} else {
				rawOdoInfo.motion.angle = 0;
				rawOdoInfo.motion.translation = 0;
				rawOdoInfo.motion.rotation = 0;
				ros::Time t = ros::Time::now();
				uint64_t timestamp = (t.sec * 1000000000UL + t.nsec);
				rawOdoInfo.timestamp = timestamp;
			}
		} else {
			// If there is a request, try to process it
			if (Motion::running) {
				this->executeRequest(request);
			}

			if (requestOld != nullptr)
				delete requestOld;

			lastCommandTimestamp = this->cycleLastTimestamp;
			requestOld = request;
		}

		calcOdoPosition();
		// send raw odometry info
		this->rawOdometryInfoPub.publish(this->rawOdoInfo);
		log_goalie();

		// minCycleTime (us), Ticks (tick), cycleLastTimestamp (tick), 1 tick = 100 ns
		long sleepTime = this->minCycleTime
				- chrono::duration_cast<chrono::milliseconds>(
						std::chrono::steady_clock::now()
								- this->cycleLastTimestamp).count();

		if (sleepTime > 0) {
			chrono::milliseconds dura(sleepTime);
			this_thread::sleep_for(dura);
		}
	}

}

void Motion::executeRequest(MotionSet* ms) {

//		trans = Math.Sign(ms.translation)*Math.Min(Math.Abs(ms.translation),this.maxVelocity);
	double trans = min(abs(ms->translation), this->maxVelocity);
	if (ms->translation < 0)
		trans *= -1;
//		rot = Math.Max(-8*Math.PI, Math.Min(8*Math.PI,ms.rotation));
	double rot = max(-8 * M_PI, min(8 * M_PI, ms->rotation));

	shared_ptr<CNMCPacketControl> packet = make_shared<CNMCPacketControl>();

	packet->setData(CNMCPacket::ControlCmd::SetMotionVector,
			(short) (cos(ms->angle) * trans), (short) (sin(ms->angle) * trans),
			(short) (rot * 64));

	sendData(packet);

// reading from motion
	auto read = readData();

// check type
	if (read->cmd == CNMCPacket::RequestCmd::PathVector
			&& read->cmdgrp == CNMCPacket::CommandGroup::RequestResponse) {
		auto data = read->getBytes();

		short x1 = read->convertByteToShort(0);
		short x2 = read->convertByteToShort(2);
		short x3 = read->convertByteToShort(4);

//			mr.angle = Math.Atan2(rawMotorValues[1],rawMotorValues[0]);
		double angle = atan2(x2, x1);
//			mr.translation = Math.Sqrt(rawMotorValues[0]*rawMotorValues[0]+rawMotorValues[1]*rawMotorValues[1]);
		double translation = sqrt(x1 * x1 + x2 * x2);
//			mr.rotation = ((double)rawMotorValues[2])/64.0;
		double rotation = (double) x3 / 64.0d;

		rawOdoInfo.motion.angle = angle;

		//workaround for faulty bytes in data
		if (!(abs(translation) > maxVelocity)) {
			rawOdoInfo.motion.translation = translation;
		}

		//workaround for faulty bytes in data
		if (!(abs(rotation) > 10)) {
			rawOdoInfo.motion.rotation = rotation;
		}

		ros::Time t = ros::Time::now();
		uint64_t timestamp = (t.sec * 1000000000UL + t.nsec);
		rawOdoInfo.timestamp = timestamp;

	}
}

void Motion::handleMotionControl(msl_actuator_msgs::MotionControlPtr mc) {
	std::lock_guard<std::mutex> lck(this->motionValueMutex);
// Create a new driver command
	if (this->motionValue == nullptr) {
		this->motionValue = new MotionSet();
	}

	this->motionValue->angle = mc->motion.angle;
	this->motionValue->translation = mc->motion.translation;
	this->motionValue->rotation = mc->motion.rotation;

// Apply the slip control if enabled
	if ((this->slipControlEnabled)
			&& (mc->motion.translation > this->slipControlMinSpeed)) {
		this->motionValue->translation *= this->slipControlFactor;
		this->motionValue->rotation *= this->slipControlFactor;
	}
}

/**
 * This is for handling Strg + C, although no ROS communication was running.
 * @param sig
 */
void Motion::pmSigintHandler(int sig) {
	cout << endl << "Motion: Caught SIGINT! Terminating ..." << endl;
	Motion::running = false;

// Call the ros signal handler method
	ros::shutdown();
}
/**
 * This is for handling SIGTERM to terminate
 * @param sig
 */
void Motion::pmSigTermHandler(int sig) {
	cout << endl << "Motion: Caught SIGTERM! Terminating ..." << endl;
	Motion::running = false;

// Call the ros signal handler method
	ros::shutdown();
}

} /* namespace msl_driver */

int main(int argc, char** argv) {
	msl_driver::Motion* motion = new msl_driver::Motion(argc, argv);
	motion->initCommunication(argc, argv);
// has to be set after Motion::initCommunication , in order to override the ROS signal handler
	signal(SIGINT, msl_driver::Motion::pmSigintHandler);
	signal(SIGTERM, msl_driver::Motion::pmSigTermHandler);
	motion->initialize();
	motion->logging_goalie_init();
	bool r = motion->open();

	if (false == r) {
		delete motion;
		ros::shutdown();

		return 1;
	}

	std::cout << "start" << std::endl;
	motion->start();
	std::cout << "operating ..." << std::endl;

	while (motion->isRunning()) {
		chrono::milliseconds dura(500);
		this_thread::sleep_for(dura);
	}

	delete motion;
}

