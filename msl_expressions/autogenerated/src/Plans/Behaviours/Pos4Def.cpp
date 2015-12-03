using namespace std;
#include "Plans/Behaviours/Pos4Def.h"

/*PROTECTED REGION ID(inccpp1445438142979) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include "engine/constraintmodul/ConstraintQuery.h"
#include "GSolver.h"
#include "SolverType.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1445438142979) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    Pos4Def::Pos4Def() :
            DomainBehaviour("Pos4Def")
    {
        /*PROTECTED REGION ID(con1445438142979) ENABLED START*/ //Add additional options here
        this->query = make_shared < alica::ConstraintQuery > (this);
        /*PROTECTED REGION END*/
    }
    Pos4Def::~Pos4Def()
    {
        /*PROTECTED REGION ID(dcon1445438142979) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void Pos4Def::run(void* msg)
    {
        /*PROTECTED REGION ID(run1445438142979) ENABLED START*/ //Add additional options here
        auto ownPos = wm->rawSensorData.getOwnPositionVision();
        if (ownPos == nullptr)
        {
            return;
        }

        MotionControl mc;
        if (query->getSolution(SolverType::GRADIENTSOLVER, runningPlan, result))
        {
            cout << "Pos4Def: FOUND a solution!" << endl;
            shared_ptr < geometry::CNPoint2D > alloTarget = make_shared < geometry::CNPoint2D
                    > (result.at(0), result.at(1));

            mc = msl::RobotMovement::moveToPointCarefully(
                    alloTarget->alloToEgo(*ownPos), make_shared < geometry::CNPoint2D > (0, 0)->alloToEgo(*ownPos),
                    100.0);
        }
        else
        {
            cout << "Pos4Def: Did not find a solution!" << endl;
        }
        send(mc);
        /*PROTECTED REGION END*/
    }
    void Pos4Def::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1445438142979) ENABLED START*/ //Add additional options here
        query->clearDomainVariables();
        query->addVariable(wm->getOwnId(), "x");
        query->addVariable(wm->getOwnId(), "y");
        result.clear();
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1445438142979) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
