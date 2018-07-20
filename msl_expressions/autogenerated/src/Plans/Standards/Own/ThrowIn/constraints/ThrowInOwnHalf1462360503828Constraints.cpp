#include "Plans/Standards/Own/ThrowIn/constraints/ThrowInOwnHalf1462360503828Constraints.h"
using namespace std;
using namespace alica;
/*PROTECTED REGION ID(ch1462360503828) ENABLED START*/
#include "AutoDiff.h"
#include "MSLConstraintBuilder.h"
#include "MSLFootballField.h"
#include "engine/constraintmodul/SolverTerm.h"
#include "engine/constraintmodul/ProblemDescriptor.h"
#include "MSLWorldModel.h"
#include "engine/RunningPlan.h"
#include "engine/Assignment.h"
#include <GeometryCalculator.h>
#include <Robots.h>
#include <Game.h>
#include <Ball.h>
/*PROTECTED REGION END*/

namespace alicaAutogenerated
{
    //Plan:ThrowInOwnHalf

    /*		
     * Tasks: 
     * - EP:1462360503830 : ExecuteStandard (1439997010902)
     * - EP:1462360607517 : ReceiveStandard (1439997023446)
     * - EP:1462360610006 : AlternativeReceive (1462360858945)
     * - EP:1462360612527 : Block (1461237765109)
     *
     * States:
     * - Align (1462360503829)
     * - GrabBall (1462360912906)
     * - Pass (1462360919387)
     * - Wait (1462360928236)
     * - AlignReceive (1462361351034)
     * - AlignAlternative (1462361358155)
     * - Block (1462361373364)
     * - ReceiveAlternative (1462363134771)
     * - Success (1462363320340)
     * - Receive (1462368095616)
     * - Successalternative (1462368161988)
     *
     * Vars:
     */

    /*
     * RuntimeCondition - (Name): ThrownInOwnHalf - RuntimeCondition
     * (ConditionString): 
     * Static Variables: []
     * Domain Variables:

     * forall agents in Block let v = [x, y] 

     */
    void Constraint1462361418213::getConstraint(shared_ptr<ProblemDescriptor> c, shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(cc1462361418213) ENABLED START*/
        msl::MSLWorldModel* wm = msl::MSLWorldModel::get();
        auto constraint = autodiff::LTConstraint::TRUE;
        auto util = autodiff::TermBuilder::constant(0);
        auto domainVariables = c->getDomainVars();

        vector < shared_ptr < TVec >> poses;
        vector < shared_ptr < TVec >> blocker;
        vector < shared_ptr < TVec >> receiver;

        vector < shared_ptr < geometry::CNPosition >> robotPositions;

        for (int quantorCount = 0; quantorCount < domainVariables->size(); quantorCount++)
        {
            for (int i = 0; i < domainVariables->at(quantorCount)->size(); i++)
            {
                c->getDomainRanges()->at(quantorCount).at(i).at(0).at(0) = -wm->field->getFieldLength() / 2;
                c->getDomainRanges()->at(quantorCount).at(i).at(0).at(1) = wm->field->getFieldLength() / 2;
                c->getDomainRanges()->at(quantorCount).at(i).at(1).at(0) = -wm->field->getFieldWidth() / 2;
                c->getDomainRanges()->at(quantorCount).at(i).at(1).at(1) = wm->field->getFieldWidth() / 2;
                auto robotPos = wm->robots->teammates.getTeamMatePosition(
                        c->getAgentsInScope()->at(quantorCount)->at(i));
                if (!robotPos)
                {
                    //safety measure in case one agent crashes
                    continue;
                }
                robotPositions.push_back(robotPos);
                auto vec = make_shared < TVec
                        > (initializer_list<shared_ptr<Term>> {dynamic_pointer_cast < autodiff::Variable
                                                                       > (domainVariables->at(quantorCount)->at(i)->at(
                                                                               0)),
                                                               dynamic_pointer_cast < autodiff::Variable
                                                                       > (domainVariables->at(quantorCount)->at(i)->at(
                                                                               1))});
                poses.push_back(vec);

                if (quantorCount == 0) // blocker
                {
                    blocker.push_back(vec);
                }
                else if (quantorCount == 1) // receiver
                {
                    receiver.push_back(vec);
                }
            }
        }

        shared_ptr < geometry::CNPoint2D > ballPose = wm->ball->getAlloBallPosition();
        if (ballPose == nullptr)
        {
            auto executerList = rp->getAssignment()->getRobotsWorkingAndFinished(1462360503830);
            cout << "ThrowInOwnHalf: Executer ID: " << executerList->front() << endl;
            ballPose = wm->robots->teammates.getTeamMatePosition(executerList->front())->getPoint();
        }

        shared_ptr < TVec > tvecBallPose = make_shared < TVec > (initializer_list<double> {ballPose->x, ballPose->y});
        auto ownGoalPos = wm->field->posOwnGoalMid();
        shared_ptr < TVec > ownGoalVec = make_shared < TVec > (initializer_list<double> {ownGoalPos->x, ownGoalPos->y});
        constraint = constraint & msl::MSLConstraintBuilder::applyRules(-1, poses);

        //Just for the case when we use this after "start" has been pressed
        if (wm->game->getSituation() == msl::Situation::Start)
        {
            constraint = constraint
                    & msl::MSLConstraintBuilder::ownPenaltyAreaDistanceExceptionRule(tvecBallPose, poses);
        }

        auto opps = wm->robots->opponents.getOpponentsAlloClustered();

        vector < shared_ptr < TVec >> blockPositions;
        vector < shared_ptr < TVec >> lessGoodBlockPositions;
        vector < shared_ptr < TVec >> blockOpponents;
        shared_ptr < geometry::CNPoint2D > nearestOpp = nullptr;
        //default nearest opp

        if (ballPose != nullptr)
            nearestOpp = make_shared < geometry::CNPoint2D > (ballPose->x - 250, ballPose->y);
        double dist = 999999999;

        if (opps != nullptr)
        {
            for (auto opp : *opps)
            {
                //is nearest to the ball?
                if (ballPose != nullptr)
                {
                    wm->field->mapOutOfOwnPenalty(opp, ballPose - opp);
                    double oDist = opp->distanceTo(ballPose);
                    if (oDist < dist)
                    {
                        nearestOpp = opp;
                        oDist = dist;
                    }
                }
                else
                {
                    opp = wm->field->mapOutOfOwnPenalty(opp);
                }

                //add blocking position
                shared_ptr < geometry::CNPoint2D > blockingPos;
                if (ballPose != nullptr)
                {
                    blockingPos = opp + (ballPose - opp)->normalize() * 700;
                }
                else
                {
                    blockingPos = make_shared < geometry::CNPoint2D > (opp->x - 700, opp->y);
                }

                //only add if opp is not close to the ball
                if (ballPose->distanceTo(blockingPos) > 1800 && ballPose->distanceTo(blockingPos) < 4000)
                {
                    blockPositions.push_back(
                            make_shared < TVec > (initializer_list<double> {blockingPos->x, blockingPos->y}));
                    blockOpponents.push_back(make_shared < TVec > (initializer_list<double> {opp->x, opp->y}));
                }
                else if (ballPose->distanceTo(blockingPos) > 1800)
                {
                    lessGoodBlockPositions.push_back(make_shared < TVec > (initializer_list<double> {blockingPos->x,
                                                                                                     blockingPos->y}));
                }

            }
        }

        if (blocker.size() > 0)
        {
            int opponents2BlockCount = blockPositions.size();
            //constraint = constraint & msl::MSLConstraintBuilder::see(tvecBallPose, true, 10000, blocker);
            for (int k = 0; k < blocker.size(); k++)
            {
                //close to block positions
                auto blockUtil = autodiff::TermBuilder::constant(0);
                //for (int j = 0; j < min(opponents2BlockCount, (int)blocker.size()); j++)
                for (int j = 0; j < opponents2BlockCount; j++)
                {
                    blockUtil = make_shared < autodiff::Max
                            > (blockUtil, 10
                                    * (1
                                            - ((ConstraintBuilder::distanceSqr(blocker.at(k), blockPositions.at(j)))
                                                    / (wm->field->getMaxDistanceSqr() * blocker.size()))));
                }

                //if we have too view blockers... take the next best opps
                if (opponents2BlockCount < blocker.size())
                {
                    for (int j = 0; j < lessGoodBlockPositions.size(); j++)
                    {
                        blockUtil = make_shared < autodiff::Max
                                > (blockUtil, 3
                                        * (1
                                                - ((ConstraintBuilder::distanceSqr(
                                                        blocker.at(k),
                                                        lessGoodBlockPositions.at(j - opponents2BlockCount)))
                                                        / (wm->field->getMaxDistanceSqr() * blocker.size()))));
                    }
                }
                util = util + blockUtil;
                //Do not block more than 4 m away from the ball
                /*constraint = constraint
                 & ConstraintBuilder::distanceSqr(blocker.at(k), tvecBallPose)
                 < autodiff::TermBuilder::constant(4000 * 4000);
                 */

                //avoid opponents!
                for (int j = 0; j < opponents2BlockCount; j++)
                {
                    auto dist2Opp = ConstraintBuilder::distanceSqr(blocker.at(k), blockOpponents.at(j));
                    constraint = constraint & (dist2Opp > autodiff::TermBuilder::constant(650 * 650));
                }
                //avoid other blockers and prefer block positions with a distance of 2m (to not block the same opponents)
                for (int j = k + 1; j < blocker.size(); j++)
                {
                    auto distBlockBlock = ConstraintBuilder::distanceSqr(blocker.at(k), blocker.at(j));
                    constraint = constraint & distBlockBlock > autodiff::TermBuilder::constant(800 * 800);
                    util = util + 5 * make_shared < autodiff::Min
                            > (autodiff::TermBuilder::constant(2000 * 2000), distBlockBlock)
                                    / autodiff::TermBuilder::constant(2000 * 2000 * blocker.size());
                }
            }
        }

        //lazy utility
        vector < shared_ptr < TVec >> realRobotPosesTVec;
        for (int i = 0; i < robotPositions.size(); i++)
        {
            auto pos = robotPositions.at(i);
            if (pos != nullptr)
                realRobotPosesTVec.push_back(make_shared < TVec > (initializer_list<double> {pos->x, pos->y}));
            else
                realRobotPosesTVec.push_back(nullptr);
        }

        util = util + msl::MSLConstraintBuilder::lazyUtil(realRobotPosesTVec, poses);
        c->setConstraint(dynamic_pointer_cast < alica::SolverTerm > (constraint));
        c->setUtility(dynamic_pointer_cast < alica::SolverTerm > (util));
        /*PROTECTED REGION END*/
    }

// State: Align

// State: Align

// State: GrabBall

// State: GrabBall

// State: Pass

// State: Pass

// State: Wait

// State: Wait

// State: AlignReceive

// State: AlignReceive

// State: AlignAlternative

// State: AlignAlternative

// State: Block

// State: Block

// State: ReceiveAlternative

// State: ReceiveAlternative

// State: Success

// State: Success

// State: Receive

// State: Receive

// State: Successalternative

// State: Successalternative

}
