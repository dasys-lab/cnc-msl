using namespace std;

#include "BehaviourCreator.h"
#include "engine/BasicBehaviour.h"

#include  "Plans/Behaviours/GetBall.h"

#include  "Plans/Behaviours/GoalKick.h"

#include  "Plans/GenericBehaviours/Stop.h"

#include  "Plans/Behaviours/Actuate.h"

#include  "Plans/Behaviours/AlineToGoal.h"

#include  "Plans/Behaviours/DriveForward.h"

#include  "Plans/GenericBehaviours/DriveToPoint.h"

#include  "Plans/Behaviours/Joystick.h"

#include  "Plans/GenericBehaviours/InterseptCarefully.h"

#include  "Plans/Behaviours/DribbleToPoint.h"

#include  "Plans/TwoHoledWall/AlignAndShootTwoHoledBall.h"

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

            case 1414828313541:

            case 1414840399972:

                return make_shared<GetBall>();
                break;

            case 1415205578139:

                return make_shared<GoalKick>();
                break;

            case 1413992626194:

                return make_shared<Stop>();
                break;

            case 1417017552846:

                return make_shared<Actuate>();
                break;

            case 1415205285582:

                return make_shared<AlineToGoal>();
                break;

            case 1417017580650:

                return make_shared<DriveForward>();
                break;

            case 1417620583364:

                return make_shared<DriveToPoint>();
                break;

            case 1421854995808:

                return make_shared<Joystick>();
                break;

            case 1417620676868:

                return make_shared<InterseptCarefully>();
                break;

            case 1414752423981:

                return make_shared<DribbleToPoint>();
                break;

            case 1417620730939:

                return make_shared<AlignAndShootTwoHoledBall>();
                break;

            default:
                cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourConfId << endl;
                throw new exception();
                break;
        }
    }
}
