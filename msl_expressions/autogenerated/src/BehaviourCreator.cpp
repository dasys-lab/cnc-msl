using namespace std;

#include "BehaviourCreator.h"
#include "engine/BasicBehaviour.h"

#include  "Plans/TwoHoledWall/AlignAndShootTwoHoledWall.h"

#include  "Plans/Attack/Wander.h"

#include  "Plans/Penalty/PenaltyAlignAndShoot.h"

#include  "Plans/Behaviours/AttackOpp.h"

#include  "Plans/Attack/OneEighty.h"

#include  "Plans/Attack/Tackle.h"

#include  "Plans/Behaviours/GoalKick.h"

#include  "Plans/GenericBehaviours/InterceptCarefully.h"

#include  "Plans/GenericBehaviours/DriveToPoint.h"

#include  "Plans/Behaviours/AlignToGoal.h"

#include  "Plans/Behaviours/ShovelSelect.h"

#include  "Plans/GenericStandards/StandardAlignToPoint.h"

#include  "Plans/GenericBehaviours/Parking.h"

#include  "Plans/Example/DriveInSquare.h"

#include  "Plans/Behaviours/GetBall.h"

#include  "Plans/GenericBehaviours/Stop.h"

#include  "Plans/Behaviours/Actuate.h"

#include  "Plans/Behaviours/DriveForward.h"

#include  "Plans/Behaviours/Joystick.h"

#include  "Plans/GenericStandards/StandardReceive.h"

#include  "Plans/Behaviours/DribbleToPoint.h"

namespace alica
{

    BehaviourCreator::BehaviourCreator()
    {
    }

    BehaviourCreator::~BehaviourCreator()
    {
    }

    shared_ptr<BasicBehaviour> BehaviourCreator::createBehaviour(long behaviourConfId)
    {
        switch (behaviourConfId)
        {

            case 1417620730939:

                return make_shared<AlignAndShootTwoHoledWall>();
                break;

            case 1434716230628:

                return make_shared<Wander>();
                break;

            case 1431531542052:

                return make_shared<PenaltyAlignAndShoot>();
                break;

            case 1430324680117:

                return make_shared<AttackOpp>();
                break;

            case 1434650910857:

                return make_shared<OneEighty>();
                break;

            case 1434807680165:

                return make_shared<Tackle>();
                break;

            case 1415205578139:

                return make_shared<GoalKick>();
                break;

            case 1427703234654:

                return make_shared<InterceptCarefully>();
                break;

            case 1417620583364:

            case 1431527260342:

                return make_shared<DriveToPoint>();
                break;

            case 1415205285582:

                return make_shared<AlignToGoal>();
                break;

            case 1434199852589:

            case 1435156714286:

            case 1435156811453:

                return make_shared<ShovelSelect>();
                break;

            case 1433950043262:

            case 1435155363994:

                return make_shared<StandardAlignToPoint>();
                break;

            case 1429111645834:

                return make_shared<Parking>();
                break;

            case 1433939634320:

                return make_shared<DriveInSquare>();
                break;

            case 1414828313541:

            case 1414840399972:

                return make_shared<GetBall>();
                break;

            case 1413992626194:

                return make_shared<Stop>();
                break;

            case 1417017552846:

                return make_shared<Actuate>();
                break;

            case 1417017580650:

                return make_shared<DriveForward>();
                break;

            case 1421854995808:

            case 1426695479346:

                return make_shared<Joystick>();
                break;

            case 1428509534191:

                return make_shared<StandardReceive>();
                break;

            case 1414752423981:

                return make_shared<DribbleToPoint>();
                break;

            default:
                cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourConfId << endl;
                throw new exception();
                break;
        }
    }
}
