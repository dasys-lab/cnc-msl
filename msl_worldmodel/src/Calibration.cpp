#include <Calibration.h>

#include "MSLWorldModel.h"
#include <process_manager/ProcessCommand.h>
#include <SystemConfig.h>

namespace msl
{

Calibration::Calibration(MSLWorldModel* wm)
{
	this->wm = wm;
	this->sc = supplementary::SystemConfig::getInstance();
	processCommandPub = n.advertise<process_manager::ProcessCommand>("/process_manager/ProcessCommand", 10);
}

Calibration::~Calibration()
{
}

double Calibration::getRobotRadius()
{
    // TODO test if this breaks anything, remove line otherwise
    //              supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
    supplementary::Configuration *motion = (*sc)["Motion"];

    return motion->get<double>("Motion", "MotionControl", "RobotRadius", NULL);
}

void Calibration::setRobotRadius(double newRadius)
{
    // TODO test if this breaks anything, remove line otherwise
    //              supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
    supplementary::Configuration *motion = (*sc)["Motion"];
    motion->set(std::to_string(newRadius), "Motion.MotionControl.RobotRadius", NULL);
    motion->store();
}

double Calibration::adjustRobotRadius(double difference)
{
    double newRadius = getRobotRadius() + difference;
    setRobotRadius(newRadius);
    return newRadius;
}

void Calibration::sendKillMotionCommand()
{
    // cout << "killing motion" << endl;
    supplementary::Configuration *processManaging = (*sc)["ProcessManaging"];

    int processId = processManaging->get<int>("Processes", "ProcessDescriptions", "Motion", "id", NULL);
    std::vector<int> ownRobotId;
    ownRobotId.push_back(this->wm->getOwnId());
    std::vector<int> pKeys;
    pKeys.push_back(processId);
    process_manager::ProcessCommand command;
    command.cmd = 1;
    command.receiverId = this->wm->getOwnId();
    command.robotIds = ownRobotId;
    command.processKeys = pKeys;
    std::vector<int> paramsets;
    paramsets.push_back(0);
    command.paramSets = paramsets;
    processCommandPub.publish(command);
}

void Calibration::sendStartMotionCommand()
{
    // cout << "starting motion" << endl;
    supplementary::Configuration *processManaging = (*sc)["ProcessManaging"];

    int processId = processManaging->get<int>("Processes", "ProcessDescriptions", "Motion", "id", NULL);
    std::vector<int> ownRobotId;
    ownRobotId.push_back(this->wm->getOwnId());
    std::vector<int> pKeys;
    pKeys.push_back(processId);
    process_manager::ProcessCommand command;
    command.cmd = 0;
    command.receiverId = this->wm->getOwnId();
    command.robotIds = ownRobotId;
    command.processKeys = pKeys;
    std::vector<int> paramsets;
    paramsets.push_back(0);
    command.paramSets = paramsets;
    processCommandPub.publish(command);
}

} /* namespace msl */
