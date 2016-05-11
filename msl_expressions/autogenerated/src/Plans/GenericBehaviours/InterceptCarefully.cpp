using namespace std;
#include "Plans/GenericBehaviours/InterceptCarefully.h"

/*PROTECTED REGION ID(inccpp1427703218101) ENABLED START*/ //Add additional includes here
#include "robotmovement/RobotMovement.h"
#include "pathplanner/PathProxy.h"
#include "pathplanner/evaluator/PathEvaluator.h"
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1427703218101) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    InterceptCarefully::InterceptCarefully() :
            DomainBehaviour("InterceptCarefully")
    {
        /*PROTECTED REGION ID(con1427703218101) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    InterceptCarefully::~InterceptCarefully()
    {
        /*PROTECTED REGION ID(dcon1427703218101) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void InterceptCarefully::run(void* msg)
    {
        /*PROTECTED REGION ID(run1427703218101) ENABLED START*/ //Add additional options here
        auto me = wm->rawSensorData->getOwnPositionVision();
        auto egoBallPos = wm->ball->getEgoBallPosition();
        MotionControl mc;
        if (me == nullptr || egoBallPos == nullptr)
        {
            send(mc);
            return;
        }

        shared_ptr < geometry::CNPoint2D > egoTarget;
        shared_ptr < geometry::CNPoint2D > egoAlignPoint;
        double snapDistance;
        shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> additionalPoints;

//		MotionControl mc;
        if (egoTarget->length() > 400)
        {
//			MSLWorldModel* wm = MSLWorldModel::get();

            shared_ptr < msl::PathEvaluator > eval = make_shared<msl::PathEvaluator>();
            shared_ptr < geometry::CNPoint2D > temp = msl::PathProxy::getInstance()->getEgoDirection(egoTarget, eval,
                                                                                                     additionalPoints);
            if (temp == nullptr)
            {
                cout << "InterceptCarefully::getEgoDirection == nullptr => ownPos not available" << endl;
                temp = egoTarget;
            }
            mc.motion.angle = temp->angleTo();
            mc.motion.rotation = egoAlignPoint->rotate(M_PI)->angleTo() * interceptCarfullyRotateP;
            if (egoTarget->length() > snapDistance)
            {
                mc.motion.translation = min(defaultTranslation, temp->length());
            }
            else
            {
                mc.motion.translation = 0;
            }
//			return mc;
        }
        else
        {
            mc.motion.angle = egoTarget->angleTo();
            mc.motion.rotation = egoAlignPoint->rotate(M_PI)->angleTo() * interceptCarfullyRotateP;
            if (egoTarget->length() > snapDistance)
            {
                mc.motion.translation = min(defaultTranslation, egoTarget->length());
            }
            else
            {
                mc.motion.translation = 0;
            }
//			return mc;
        }

//        mc = msl::RobotMovement::interceptCarefully(egoBallPos, egoBallPos, 100, nullptr);

        send(mc);
        /*PROTECTED REGION END*/
    }
    void InterceptCarefully::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1427703218101) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    /*PROTECTED REGION ID(methods1427703218101) ENABLED START*/ //Add additional methods here
    void InterceptCarefully::readConfigParameters()
    {
        interceptCarfullyRotateP = (*sc)["Drive"]->get<double>("Drive.Carefully.RotateP", NULL);
        defaultTranslation = (*sc)["Drive"]->get<double>("Drive.Default.Velocity", NULL);
    }

/*PROTECTED REGION END*/
} /* namespace alica */
