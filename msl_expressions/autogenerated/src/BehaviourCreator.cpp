using namespace std;

#include "BehaviourCreator.h"
#include "engine/BasicBehaviour.h"

#include  "Plans/Attack/SearchForPassPoint.h"

#include  "Plans/Behaviours/MoveToPointDynamic.h"

#include  "Plans/TwoHoledWall/AlignAndShootTwoHoledWall.h"

#include  "Plans/Penalty/PenaltyAlignAndShoot.h"

#include  "Plans/Behaviours/AttackOpp.h"

#include  "Plans/Behaviours/GoalKick.h"

#include  "Plans/Behaviours/PositionReceiver.h"

#include  "Plans/Standards/Own/PassIntoPath/StandardAlignToPassPos.h"

#include  "Plans/GenericBehaviours/DriveToPoint.h"

#include  "Plans/GenericStandards/StandardAlignAndGrab2Receivers.h"

#include  "Plans/Attack/DribbleToAttackPointConservative.h"

#include  "Plans/Standards/Own/Corner/BouncePassShoot.h"

#include  "Plans/GenericStandards/StandardAlignToPoint.h"

#include  "Plans/Behaviours/ActuatorPassTest.h"

#include  "Plans/Attack/DribbleEmergencyKick.h"

#include  "Plans/GenericBehaviours/Stop.h"

#include  "Plans/Behaviours/Joystick.h"

#include  "Plans/Behaviours/RobotMovementDribbleTest.h"

#include  "Plans/Dribble/DribbleControl.h"

#include  "Plans/Attack/AlignAndPassRapid.h"

#include  "Plans/Behaviours/PositionExecutor.h"

#include  "Plans/Behaviours/Pos2Defenders.h"

#include  "Plans/GenericBehaviours/CheckGoalKick.h"

#include  "Plans/Penalty/DriveToPenaltyStart.h"

#include  "Plans/Behaviours/Duel.h"

#include  "Plans/Defence/ReleaseMid.h"

#include  "Plans/Standards/Own/FreeKick/PositionReceiverFreeKickOppHalf.h"

#include  "Plans/Standards/Own/Penalty/PenaltyPosExecuter.h"

#include  "Plans/Goalie/Test/GoalieBehaviours/DriveToGoal.h"

#include  "Plans/Attack/DribbleToAttackPoint.h"

#include  "Plans/Goalie/Test/GoalieBehaviours/WatchBall.h"

#include  "Plans/GenericStandards/StandardPass.h"

#include  "Plans/Behaviours/AlignToGoal.h"

#include  "Plans/Behaviours/Pos4Def.h"

#include  "Plans/TestPlans/GoalieMotionTuning/DriveToPost.h"

#include  "Plans/Standards/Opponent/Penalty/Pos4OppPenaltyIntercept.h"

#include  "Plans/Behaviours/ShovelSelect.h"

#include  "Plans/Standards/Own/PassIntoPath/StandardAlignToGeneric.h"

#include  "Plans/Attack/ProtectBall.h"

#include  "Plans/GenericBehaviours/Parking.h"

#include  "Plans/Standards/Own/Penalty/AfterGame/StayAwayAfterGamePenalty.h"

#include  "Plans/Behaviours/DriveForward.h"

#include  "Plans/Behaviours/CheckPassMsg.h"

#include  "Plans/GameStrategy/Other/CoverSpace.h"

#include  "Plans/Standards/Own/FreeKick/AlignFreeGoalSpace.h"

#include  "Plans/TestPlans/KickCurveTuning/LaserBallTracking.h"

#include  "Plans/Standards/Own/ThrowIn/ThrowInPass.h"

#include  "Plans/GenericStandards/GenericExecutePass.h"

#include  "Plans/Attack/Wander.h"

#include  "Plans/Behaviours/DefendGoal.h"

#include  "Plans/Behaviours/Intercept.h"

#include  "Plans/Standards/Own/Corner/BounceShotAlignWall.h"

#include  "Plans/Standards/Own/ThrowIn/PosAlternativePassReceiver.h"

#include  "Plans/Standards/Own/FreeKick/Pos2Penalty.h"

#include  "Plans/Standards/Own/Penalty/InGame/Pos4PenaltyRebounce.h"

#include  "Plans/Standards/Own/ThrowIn/PositionAlternativeReceiver.h"

#include  "Plans/Behaviours/KickOffPassDefault.h"

#include  "Plans/Behaviours/CalcCalib.h"

#include  "Plans/Attack/FetchFromSideLine.h"

#include  "Plans/Behaviours/StdExecutorGrabBall.h"

#include  "Plans/Behaviours/GoalieExtension.h"

#include  "Plans/GenericStandards/StandardAlignToPoint2Receivers.h"

#include  "Plans/Standards/Own/ThrowIn/ReceiveInOppHalf.h"

#include  "Plans/Goalie/Test/GoalieBehaviours/BlockBall.h"

#include  "Plans/Standards/Own/ThrowIn/PositionReceiverThrownIn.h"

#include  "Plans/Standards/Own/PassIntoPath/ReceivePassIntoPathGeneric.h"

#include  "Plans/GenericStandards/StandardReceive.h"

#include  "Plans/Attack/DribbleAttackConservative.h"

#include  "Plans/Behaviours/DribbleToPoint.h"

#include  "Plans/Attack/AdvancdeSimplePass.h"

#include  "Plans/Attack/OneEighty.h"

#include  "Plans/Standards/Own/Penalty/PenaltyShoot.h"

#include  "Plans/Defence/OneGernericInGameBlocker.h"

#include  "Plans/Behaviours/BackroomDefence.h"

#include  "Plans/GenericBehaviours/InterceptCarefully.h"

#include  "Plans/Standards/Own/Corner/BounceShotAlignPasser.h"

#include  "Plans/Attack/CatchPass.h"

#include  "Plans/Behaviours/SpinSlowly.h"

#include  "Plans/Goalie/Test/GoalieBehaviours/DriveToBall.h"

#include  "Plans/GenericStandards/StandardActuate.h"

#include  "Plans/Robotcheck/RobotTest.h"

#include  "Plans/Standards/Opponent/Penalty/Pos4OppPenaltyRebounce.h"

#include  "Plans/GameStrategy/Other/DropBallAttackerPos.h"

#include  "Plans/Standards/Own/PassIntoPath/PassKickIntoPath.h"

#include  "Plans/Behaviours/GetBall.h"

#include  "Plans/Standards/Own/Corner/BouncePassFinishAlign.h"

#include  "Plans/Standards/Opponent/TeamWatchBall.h"

#include  "Plans/Standards/Own/Corner/StandardDefendPos.h"

#include  "Plans/GenericStandards/StandardAlignAndGrab.h"

#include  "Plans/Goalie/Test/GoalieBehaviours/KickToDirection.h"

#include  "Plans/Standards/Own/Corner/Pos4ReceiverCornerKick.h"

#include  "Plans/GenericStandards/StandardShieldBall.h"

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

            case 1436269036396:

            case 1441107270872:

                return make_shared<SearchForPassPoint>();
                break;

            case 1456997097907:

            case 1458033795798:

                return make_shared<MoveToPointDynamic>();
                break;

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

            case 1439379352605:

                return make_shared<PositionReceiver>();
                break;

            case 1457532300654:

                return make_shared<StandardAlignToPassPos>();
                break;

            case 1417620583364:

            case 1431527260342:

            case 1442921078802:

            case 1443003717671:

            case 1436961206415:

                return make_shared<DriveToPoint>();
                break;

            case 1462368748899:

                return make_shared<StandardAlignAndGrab2Receivers>();
                break;

            case 1458132905432:

                return make_shared<DribbleToAttackPointConservative>();
                break;

            case 1459357188003:

                return make_shared<BouncePassShoot>();
                break;

            case 1433950043262:

            case 1435155363994:

                return make_shared<StandardAlignToPoint>();
                break;

            case 1467309206305:

                return make_shared<ActuatorPassTest>();
                break;

            case 1457706826895:

            case 1457706895442:

                return make_shared<DribbleEmergencyKick>();
                break;

            case 1413992626194:

                return make_shared<Stop>();
                break;

            case 1421854995808:

            case 1426695479346:

                return make_shared<Joystick>();
                break;

            case 1462969753310:

                return make_shared<RobotMovementDribbleTest>();
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

            case 1444835591397:

                return make_shared<Pos2Defenders>();
                break;

            case 1449076029919:

            case 1467265292648:

                return make_shared<CheckGoalKick>();
                break;

            case 1459609537461:

                return make_shared<DriveToPenaltyStart>();
                break;

            case 1450178707835:

                return make_shared<Duel>();
                break;

            case 1458033497042:

                return make_shared<ReleaseMid>();
                break;

            case 1464780824372:

                return make_shared<PositionReceiverFreeKickOppHalf>();
                break;

            case 1466940432683:

                return make_shared<PenaltyPosExecuter>();
                break;

            case 1447863442558:

                return make_shared<DriveToGoal>();
                break;

            case 1436855860607:

            case 1437391438054:

                return make_shared<DribbleToAttackPoint>();
                break;

            case 1447863472667:

                return make_shared<WatchBall>();
                break;

            case 1435760175843:

                return make_shared<StandardPass>();
                break;

            case 1415205285582:

                return make_shared<AlignToGoal>();
                break;

            case 1445438204426:

                return make_shared<Pos4Def>();
                break;

            case 1464189840525:

                return make_shared<DriveToPost>();
                break;

            case 1466975764775:

                return make_shared<Pos4OppPenaltyIntercept>();
                break;

            case 1435156714286:

            case 1435156811453:

                return make_shared<ShovelSelect>();
                break;

            case 1457531639350:

                return make_shared<StandardAlignToGeneric>();
                break;

            case 1457706612268:

                return make_shared<ProtectBall>();
                break;

            case 1429111645834:

                return make_shared<Parking>();
                break;

            case 1466940600117:

                return make_shared<StayAwayAfterGamePenalty>();
                break;

            case 1417017580650:

                return make_shared<DriveForward>();
                break;

            case 1457441499013:

                return make_shared<CheckPassMsg>();
                break;

            case 1455537928849:

            case 1455537979559:

                return make_shared<CoverSpace>();
                break;

            case 1467039882734:

                return make_shared<AlignFreeGoalSpace>();
                break;

            case 1457698689219:

                return make_shared<LaserBallTracking>();
                break;

            case 1462363309950:

                return make_shared<ThrowInPass>();
                break;

            case 1465040471344:

                return make_shared<GenericExecutePass>();
                break;

            case 1434716230628:

                return make_shared<Wander>();
                break;

            case 1459249336699:

                return make_shared<DefendGoal>();
                break;

            case 1458757193843:

                return make_shared<Intercept>();
                break;

            case 1459355025721:

            case 1459356753335:

                return make_shared<BounceShotAlignWall>();
                break;

            case 1461674968023:

                return make_shared<PosAlternativePassReceiver>();
                break;

            case 1465474190742:

                return make_shared<Pos2Penalty>();
                break;

            case 1466975991599:

            case 1466976004315:

                return make_shared<Pos4PenaltyRebounce>();
                break;

            case 1462978671719:

                return make_shared<PositionAlternativeReceiver>();
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

            case 1459249287791:

                return make_shared<GoalieExtension>();
                break;

            case 1467229016494:

                return make_shared<StandardAlignToPoint2Receivers>();
                break;

            case 1462370388995:

                return make_shared<ReceiveInOppHalf>();
                break;

            case 1447863463711:

                return make_shared<BlockBall>();
                break;

            case 1461584235418:

                return make_shared<PositionReceiverThrownIn>();
                break;

            case 1457531594373:

                return make_shared<ReceivePassIntoPathGeneric>();
                break;

            case 1428509534191:

                return make_shared<StandardReceive>();
                break;

            case 1457967385543:

                return make_shared<DribbleAttackConservative>();
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

            case 1466940268216:

                return make_shared<PenaltyShoot>();
                break;

            case 1458034300406:

                return make_shared<OneGernericInGameBlocker>();
                break;

            case 1454507819086:

                return make_shared<BackroomDefence>();
                break;

            case 1427703234654:

                return make_shared<InterceptCarefully>();
                break;

            case 1459354990329:

            case 1459357015987:

                return make_shared<BounceShotAlignPasser>();
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

            case 1456756164754:

                return make_shared<RobotTest>();
                break;

            case 1466975723661:

                return make_shared<Pos4OppPenaltyRebounce>();
                break;

            case 1455537879822:

                return make_shared<DropBallAttackerPos>();
                break;

            case 1457531685581:

                return make_shared<PassKickIntoPath>();
                break;

            case 1414828313541:

            case 1414840399972:

                return make_shared<GetBall>();
                break;

            case 1459357089325:

                return make_shared<BouncePassFinishAlign>();
                break;

            case 1457015565562:

                return make_shared<TeamWatchBall>();
                break;

            case 1459355071258:

            case 1459356685875:

                return make_shared<StandardDefendPos>();
                break;

            case 1455888617961:

            case 1459456566595:

            case 1461583806472:

            case 1465038982091:

            case 1466861369486:

                return make_shared<StandardAlignAndGrab>();
                break;

            case 1447863487000:

                return make_shared<KickToDirection>();
                break;

            case 1464787549220:

                return make_shared<Pos4ReceiverCornerKick>();
                break;

            case 1455888715611:

                return make_shared<StandardShieldBall>();
                break;

            default:
                cerr << "BehaviourCreator: Unknown behaviour requested: " << behaviourConfId << endl;
                throw new exception();
                break;
        }
    }
}
