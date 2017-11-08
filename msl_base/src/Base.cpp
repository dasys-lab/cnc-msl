#include "Base.h"

#include <CGSolver.h>
#include <SigFault.h>
#include <SolverType.h>
#include <clock/AlicaROSClock.h>
#include <communication/AlicaRosCommunication.h>
#include <msl_robot/robotmovement/RobotMovement.h>
#include <supplementary/AgentIDManager.h>
#include <msl/robot/IntRobotIDFactory.h>

#include <ros/ros.h>

#include <chrono>
#include <thread>
#include <iostream>

using namespace std;
using namespace msl;

namespace msl
{

Base::Base(string roleSetName, string masterPlanName, string roleSetDir, bool sim)
{
    ae = new alica::AlicaEngine();
    bc = new alica::BehaviourCreator();
    cc = new alica::ConditionCreator();
    uc = new alica::UtilityFunctionCreator();
    crc = new alica::ConstraintCreator();
    auto factory = new msl::robot::IntRobotIDFactory();
    idManager = new supplementary::AgentIDManager(factory);
    ae->setIAlicaClock(new alicaRosProxy::AlicaROSClock());
    ae->setCommunicator(new alicaRosProxy::AlicaRosCommunication(ae));
    if (sim)
    {
        cout << "Base Vorher: " << ae->getIAlicaClock()->now() << endl;
        ae->getIAlicaClock()->sleep(200000);
        cout << "Base Nachher: " << ae->getIAlicaClock()->now() << endl;
    }

    ae->addSolver(SolverType::GRADIENTSOLVER, new alica::reasoner::CGSolver(ae));
    ae->init(bc, cc, uc, crc, idManager, roleSetName, masterPlanName, roleSetDir, false);

    wm = MSLWorldModel::get();
    if (sim)
    {
        wm->timeLastSimMsgReceived = 1;
    }
    wm->setEngine(ae);
}

void Base::start()
{
    ae->start();
}

Base::~Base()
{
    ae->shutdown();
    delete ae->getIAlicaClock();
    delete ae->getCommunicator();
    delete ae;
    delete cc;
    delete bc;
    delete uc;
    delete crc;
    delete idManager;
}

} /* namespace msl */

void printUsage()
{
    cout << "Usage: ./msl_base -m \"Masterplan\" [-rd \"RoleSetDirectory\"] [-rset \"RoleSet\"] [-sim]" << endl;
}

string getNodeName(string postFix)
{
    string hostName = supplementary::SystemConfig::getInstance()->getHostname();
    replace(hostName.begin(), hostName.end(), '-', '_');

    return hostName + "_" + postFix;
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        printUsage();
        return 0;
    }
    cout << "Initialising ROS" << endl;
    ros::init(argc, argv, getNodeName("Base"));
    // This makes segfaults to exceptions
    segfaultdebug::init_segfault_exceptions();
    cout << "Parsing command line parameters:" << endl;
    string masterplan = "";
    string rolesetdir = ".";
    string roleset = "";
    bool sim = false;
    for (int i = 1; i < argc; i++)
    {
        if (string(argv[i]) == "-m" || string(argv[i]) == "-masterplan")
        {
            masterplan = argv[i + 1];
            i++;
        }

        if (string(argv[i]) == "-rd" || string(argv[i]) == "-rolesetdir")
        {
            rolesetdir = argv[i + 1];
            i++;
        }
        if (string(argv[i]) == "-r" || string(argv[i]) == "-roleset")
        {
            roleset = argv[i + 1];
            i++;
        }
        if (string(argv[i]) == "-sim")
        {
            sim = true;
        }
    }
    if (masterplan.size() == 0 || rolesetdir.size() == 0)
    {
        printUsage();
        return 0;
    }
    cout << "\tMasterplan is:       \"" << masterplan << "\"" << endl;
    cout << "\tRolset Directory is: \"" << rolesetdir << "\"" << endl;
    cout << "\tRolset is:           \"" << (roleset.empty() ? "Default" : roleset) << "\"" << endl;

    cout << "\nConstructing Base ..." << endl;
    Base *base = new Base(roleset, masterplan, rolesetdir, sim);

    cout << "\nStarting Base ..." << endl;
    base->start();

    while (ros::ok())
    {
        std::chrono::milliseconds dura(500);
        std::this_thread::sleep_for(dura);
    }
    delete base;

    return 0;
}
