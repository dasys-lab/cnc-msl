#include "ConstraintCreator.h"
#include <iostream>

#include  "Plans/GameStrategy/Other/constraints/Parking1426695119330Constraints.h"

#include  "Plans/Attack/constraints/Dribble1434049476066Constraints.h"

#include  "Plans/GenericStandards/constraints/GenericOppStandards1432132075122Constraints.h"

#include  "Plans/Example/constraints/ExamplePlan1433938652021Constraints.h"

#include  "Plans/GenericStandards/constraints/GenericExecute1431522123418Constraints.h"

#include  "Plans/Attack/constraints/StandardAttack1434046634656Constraints.h"

#include  "Plans/Attack/TestPlans/constraints/DribbleToPointPlan1436960829485Constraints.h"

#include  "Plans/Attack/constraints/AttackPlay1434045709191Constraints.h"

#include  "Plans/Attack/constraints/RunFree1434115664325Constraints.h"

#include  "Plans/Attack/TestPlans/constraints/TestPassPointMaster1441106724156Constraints.h"

#include  "Plans/constraints/WM161413992564408Constraints.h"

#include  "Plans/TwoHoledWall/constraints/TwoHoledWallMaster1417621468963Constraints.h"

#include  "Plans/Attack/TestPlans/constraints/PassPlan1441106995954Constraints.h"

#include  "Plans/Attack/TestPlans/constraints/TestAttackPlan1436960675873Constraints.h"

#include  "Plans/Attack/constraints/AttackSupportPlan1434046705214Constraints.h"

#include  "Plans/Attack/TestPlans/constraints/TestDribblePlan1437902404050Constraints.h"

#include  "Plans/GameStrategy/Gameplay/constraints/Gameplay1426694853089Constraints.h"

#include  "Plans/GameStrategy/Other/constraints/SimpleDropBall1426696586622Constraints.h"

#include  "Plans/Standards/Opponent/FreeKick/constraints/OppFreeKick1445411471122Constraints.h"

#include  "Plans/Defence/Test/constraints/TestApproachBallMaster1430324312981Constraints.h"

#include  "Plans/Standards/Own/KickOff/constraints/OwnKickOff1438785376159Constraints.h"

#include  "Plans/ActuatorTest/constraints/ActuatorTestMaster1417017436952Constraints.h"

#include  "Plans/GameStrategy/Other/constraints/DroppedBall1426694906399Constraints.h"

#include  "Plans/Attack/constraints/PassPlay1436268896671Constraints.h"

#include  "Plans/TwoHoledWall/constraints/ShootTwoHoledWall1417620189234Constraints.h"

<<<<<<< HEAD
#include  "Plans/Calibration/constraints/MotionCalibration1442919721161Constraints.h"
=======
#include  "Plans/Attack/constraints/StandardKickOff1438777024734Constraints.h"
>>>>>>> a15bdeedf6f216132146f89a91075152d997eee2

#include  "Plans/GenericStandards/constraints/GenericDefend1432133473779Constraints.h"

#include  "Plans/Penalty/constraints/OwnPenalty1431525185678Constraints.h"

#include  "Plans/Attack/constraints/Tackle1434116965565Constraints.h"

#include  "Plans/GenericStandards/constraints/GenericOwnStandards1430924951132Constraints.h"

#include  "Plans/GenericStandards/constraints/DummyMasterPlan1432139066765Constraints.h"

<<<<<<< HEAD
#include  "Plans/GenericStandards/constraints/DummyMasterPlan1432139066765Constraints.h"
=======
#include  "Plans/constraints/CarpetCalibrator1435159127771Constraints.h"

#include  "Plans/Attack/TestPlans/constraints/AttackOppGoalPlan1437902649389Constraints.h"
>>>>>>> a15bdeedf6f216132146f89a91075152d997eee2

#include  "Plans/Defence/constraints/MidFieldDefense1441108604411Constraints.h"

using namespace std;
using namespace alicaAutogenerated;

namespace alica
{

    ConstraintCreator::ConstraintCreator()
    {
    }

    ConstraintCreator::~ConstraintCreator()
    {
    }

    shared_ptr<BasicConstraint> ConstraintCreator::createConstraint(long constraintConfId)
    {
        switch (constraintConfId)
        {

            case 1445442215438:
                return make_shared<Constraint1445442215438>();
                break;

            default:
                cerr << "ConstraintCreator: Unknown constraint requested: " << constraintConfId << endl;
                throw new exception();
                break;
        }
    }

}
