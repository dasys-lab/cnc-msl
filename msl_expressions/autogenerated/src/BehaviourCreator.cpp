using namespace std;

#include "BehaviourCreator.h"
#include "engine/BasicBehaviour.h"

#include  "Plans/TwoHoledWall/AlignAndShootTwoHoledWall.h"

#include  "Plans/Penalty/PenaltyAlignAndShoot.h"

#include  "Plans/Behaviours/AttackOpp.h"

#include  "Plans/Behaviours/GoalKick.h"

#include  "Plans/GenericBehaviours/StandardReceive.h"

#include  "Plans/GenericBehaviours/StandardWatcherPositioningDefault.h"

#include  "Plans/GenericBehaviours/DriveToPoint.h"

#include  "Plans/GenericBehaviours/InterceptCarefully.h"

#include  "Plans/Behaviours/AlignToGoal.h"

#include  "Plans/GenericBehaviours/StandardStdDefendPositioning.h"

#include  "Plans/GenericBehaviours/StdStandardDefendPos.h"

#include  "Plans/GenericBehaviours/Parking.h"

#include  "Plans/GenericBehaviours/StandardBlockerPositioning.h"

#include  "Plans/Example/DriveInSquare.h"

#include  "Plans/Behaviours/GetBall.h"

#include  "Plans/GenericBehaviours/Stop.h"

#include  "Plans/Behaviours/Actuate.h"

#include  "Plans/Behaviours/DriveForward.h"

#include  "Plans/GenericBehaviours/StdReceiverPos.h"

#include  "Plans/Behaviours/Joystick.h"

#include  "Plans/GenericBehaviours/StdExecuterPos.h"

#include  "Plans/GenericBehaviours/StandardAlignAndShoot.h"

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

            case 1431531542052:

                return make_shared<PenaltyAlignAndShoot>();
                break;

            case 1430324680117:

                return make_shared<AttackOpp>();
                break;

            case 1415205578139:

                return make_shared<GoalKick>();
                break;

            case 1428509534191:

                return make_shared<StandardReceive>();
                break;

            case 1429109434270:

                return make_shared<StandardWatcherPositioningDefault>();
                break;

            case 1417620583364:

            case 1431527260342:

                return make_shared<DriveToPoint>();
                break;

            case 1427703234654:

                return make_shared<InterceptCarefully>();
                break;

            case 1415205285582:

                return make_shared<AlignToGoal>();
                break;

            case 1429110549548:

                return make_shared<StandardStdDefendPositioning>();
                break;

            case 1428508259449:

                return make_shared<StdStandardDefendPos>();
                break;

            case 1429111645834:

                return make_shared<Parking>();
                break;

            case 1429109488432:

                return make_shared<StandardBlockerPositioning>();
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

            case 1428508056340:

                return make_shared<StdReceiverPos>();
                break;

            case 1421854995808:

            case 1426695479346:

                return make_shared<Joystick>();
                break;

            case 1428508127438:

                return make_shared<StdExecuterPos>();
                break;

            case 1428509031167:

                return make_shared<StandardAlignAndShoot>();
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
