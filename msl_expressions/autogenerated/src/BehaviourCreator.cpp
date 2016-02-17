using namespace std;

#include "BehaviourCreator.h"
#include "engine/BasicBehaviour.h"

#include  "Plans/TwoHoledWall/AlignAndShootTwoHoledWall.h"

#include  "Plans/Attack/SearchForPassPoint.h"

#include  "Plans/Penalty/PenaltyAlignAndShoot.h"

#include  "Plans/Behaviours/AttackOpp.h"

#include  "Plans/Behaviours/PositionReceiver.h"

#include  "Plans/Behaviours/GoalKick.h"

#include  "Plans/GenericBehaviours/DriveToPoint.h"

#include  "Plans/GenericStandards/StandardAlignToPoint.h"

#include  "Plans/Example/DriveInSquare.h"

#include  "Plans/GenericBehaviours/Stop.h"

#include  "Plans/Behaviours/Actuate.h"

#include  "Plans/Behaviours/Joystick.h"

#include  "Plans/Dribble/DribbleControl.h"

#include  "Plans/Attack/AlignAndPassRapid.h"

#include  "Plans/Behaviours/PositionExecutor.h"

#include  "Plans/GenericBehaviours/CheckGoalKick.h"

#include  "Plans/Behaviours/Pos2Defenders.h"

#include  "Plans/Behaviours/Duel.h"

#include  "Plans/Goalie/Test/GoalieBehaviours/DriveToGoal.h"

#include  "Plans/Goalie/Test/GoalieBehaviours/WatchBall.h"

#include  "Plans/GenericStandards/StandardPass.h"

#include  "Plans/Attack/DribbleToAttackPoint.h"

#include  "Plans/Behaviours/AlignToGoal.h"

#include  "Plans/Behaviours/Pos4Def.h"

#include  "Plans/Behaviours/ShovelSelect.h"

#include  "Plans/GenericBehaviours/Parking.h"

#include  "Plans/Behaviours/DriveForward.h"

#include  "Plans/GameStrategy/Other/CoverSpace.h"

#include  "Plans/Behaviours/AlignToRobot.h"

#include  "Plans/Attack/Wander.h"

#include  "Plans/Behaviours/KickOffPassDefault.h"

#include  "Plans/Behaviours/CalcCalib.h"

#include  "Plans/Attack/FetchFromSideLine.h"

#include  "Plans/Behaviours/StdExecutorGrabBall.h"

#include  "Plans/Behaviours/AlignExecutor.h"

#include  "Plans/Goalie/Test/GoalieBehaviours/BlockBall.h"

#include  "Plans/GenericStandards/StandardReceive.h"

#include  "Plans/Behaviours/DribbleToPoint.h"

#include  "Plans/Attack/AdvancdeSimplePass.h"

#include  "Plans/Attack/OneEighty.h"

#include  "Plans/Attack/Tackle.h"

#include  "Plans/GenericBehaviours/InterceptCarefully.h"

#include  "Plans/Behaviours/BackroomDefence.h"

#include  "Plans/Attack/CatchPass.h"

#include  "Plans/Behaviours/SpinSlowly.h"

#include  "Plans/Goalie/Test/GoalieBehaviours/DriveToBall.h"

#include  "Plans/GenericStandards/StandardActuate.h"

#include  "Plans/GameStrategy/Other/DropBallAttackerPos.h"

#include  "Plans/Behaviours/GetBall.h"

#include  "Plans/Goalie/Test/GoalieBehaviours/KickToDirection.h"

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

            case 1436269036396:

            case 1441107270872:

                return make_shared<SearchForPassPoint>();
                break;

            case 1431531542052:

                return make_shared<PenaltyAlignAndShoot>();
                break;

            case 1430324680117:

                return make_shared<AttackOpp>();
                break;

            case 1439379352605:

                return make_shared<PositionReceiver>();
                break;

            case 1415205578139:

                return make_shared<GoalKick>();
                break;

            case 1417620583364:

            case 1431527260342:

            case 1442921078802:

            case 1443003717671:

            case 1436961206415:

                return make_shared<DriveToPoint>();
                break;

            case 1433950043262:

            case 1435155363994:

                return make_shared<StandardAlignToPoint>();
                break;

            case 1433939634320:

                return make_shared<DriveInSquare>();
                break;

            case 1413992626194:

                return make_shared<Stop>();
                break;

            case 1417017552846:

                return make_shared<Actuate>();
                break;

            case 1421854995808:

            case 1426695479346:

                return make_shared<Joystick>();
                break;

            case 1449742099555:

            case 1450175539163:

                return make_shared<DribbleControl>();
                break;

            case 1436269080263:

            case 1441108023281:

                return make_shared<AlignAndPassRapid>();
                break;

            case 1438790487994:

                return make_shared<PositionExecutor>();
                break;

            case 1449076029919:

                return make_shared<CheckGoalKick>();
                break;

            case 1444835591397:

                return make_shared<Pos2Defenders>();
                break;

            case 1450178707835:

                return make_shared<Duel>();
                break;

            case 1447863442558:

                return make_shared<DriveToGoal>();
                break;

            case 1447863472667:

                return make_shared<WatchBall>();
                break;

            case 1435760175843:

                return make_shared<StandardPass>();
                break;

            case 1436855860607:

            case 1437391438054:

                return make_shared<DribbleToAttackPoint>();
                break;

            case 1415205285582:

                return make_shared<AlignToGoal>();
                break;

            case 1445438204426:

                return make_shared<Pos4Def>();
                break;

            case 1434199852589:

            case 1435156714286:

            case 1435156811453:

                return make_shared<ShovelSelect>();
                break;

            case 1429111645834:

                return make_shared<Parking>();
                break;

            case 1417017580650:

                return make_shared<DriveForward>();
                break;

            case 1455537928849:

            case 1455537979559:

                return make_shared<CoverSpace>();
                break;

            case 1438779292567:

                return make_shared<AlignToRobot>();
                break;

            case 1434716230628:

                return make_shared<Wander>();
                break;

            case 1438778223495:

                return make_shared<KickOffPassDefault>();
                break;

            case 1446033354004:

            case 1446036332071:

            case 1446036372245:

            case 1446036391317:

                return make_shared<CalcCalib>();
                break;

            case 1450175679178:

                return make_shared<FetchFromSideLine>();
                break;

            case 1441209089978:

                return make_shared<StdExecutorGrabBall>();
                break;

            case 1440600507552:

                return make_shared<AlignExecutor>();
                break;

            case 1447863463711:

                return make_shared<BlockBall>();
                break;

            case 1428509534191:

                return make_shared<StandardReceive>();
                break;

            case 1414752423981:

                return make_shared<DribbleToPoint>();
                break;

            case 1450176216458:

                return make_shared<AdvancdeSimplePass>();
                break;

            case 1434650910857:

                return make_shared<OneEighty>();
                break;

            case 1434807680165:

                return make_shared<Tackle>();
                break;

            case 1427703234654:

                return make_shared<InterceptCarefully>();
                break;

            case 1454507819086:

                return make_shared<BackroomDefence>();
                break;

            case 1440754543898:

                return make_shared<CatchPass>();
                break;

            case 1435159282996:

                return make_shared<SpinSlowly>();
                break;

            case 1447863503279:

                return make_shared<DriveToBall>();
                break;

            case 1435766278023:

                return make_shared<StandardActuate>();
                break;

            case 1455537879822:

                return make_shared<DropBallAttackerPos>();
                break;

            case 1414828313541:

            case 1414840399972:

                return make_shared<GetBall>();
                break;

            case 1447863487000:

                return make_shared<KickToDirection>();
                break;

            default:
                cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourConfId << endl;
                throw new exception();
                break;
        }
    }
}
