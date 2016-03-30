#include "Plans/Attack/constraints/AttackPlay1434045709191Constraints.h"
using namespace std;
using namespace alica;
/*PROTECTED REGION ID(ch1434045709191) ENABLED START*/
//Add additional using directives here
#include "MSLFootballField.h"
#include "MSLWorldModel.h"
#include "engine/constraintmodul/ConstraintDescriptor.h"
#include "engine/RunningPlan.h"
#include "engine/Assignment.h"
#include "AutoDiff.h"
#include "MSLConstraintBuilder.h"

/*PROTECTED REGION END*/

namespace alicaAutogenerated
{
    //Plan:AttackPlay

    /*		
     * Tasks: 
     * - EP:1434045709194 : Attack (1222613952469)
     * - EP:1434045719840 : Blocker (1432209050494)
     * - EP:1434045723977 : Defend (1225115406909)
     * - EP:1434112675755 : InGamePassReceiver (1307185798142)
     *
     * States:
     * - Attack (1434045709193)
     * - MidFieldDefense (1434045868018)
     * - Defend (1434045870617)
     * - Release (1434112762535)
     * - ApproachPass (1436536084172)
     * - InterceptPass (1436536085953)
     * - InterceptPass (1436536121614)
     * - ApproachPass (1436536123918)
     *
     * Vars:				
     * - TargetX (1457002241973) 				
     * - TargetY (1457002247256) 
     */

    /*
     * RuntimeCondition - (Name): NewRuntimeCondition
     * (ConditionString): 
     * Static Variables: [TargetX, TargetY]
     * Domain Variables:

     */
    void Constraint1434112519736::getConstraint(shared_ptr<ConstraintDescriptor> c, shared_ptr<RunningPlan> rp)
    {
        /*PROTECTED REGION ID(cc1434112519736) ENABLED START*/
//        cout << "Attackplay: Constraint called" << endl;
        msl::MSLFootballField* ff = msl::MSLFootballField::getInstance();
        msl::MSLWorldModel* wm = msl::MSLWorldModel::get();
        auto util = autodiff::TermBuilder::constant(0);
        auto constraint = autodiff::LTConstraint::TRUE;
        c->getStaticRanges()->at(0).at(0) = -msl::MSLFootballField::FieldLength / 2 + 2000;
        c->getStaticRanges()->at(0).at(1) = msl::MSLFootballField::FieldLength / 2 - ff->PenaltyAreaLength * 1.25;
        c->getStaticRanges()->at(1).at(0) = -msl::MSLFootballField::FieldWidth / 2 + 1200;
        c->getStaticRanges()->at(1).at(1) = msl::MSLFootballField::FieldWidth / 2 - 1200;

        //Get Position of passing robot...
        shared_ptr<vector<int>> r1 = rp->getAssignment()->getRobotsWorking(1434045709194);
        shared_ptr < autodiff::TVec > passer = nullptr;
        shared_ptr < geometry::CNPosition > passerPos = nullptr;
        if (r1->size() == 0 || r1 == nullptr)
        {
            cout << "Attackplay: No Passer Assigned!?!?!?" << endl;
            passer = make_shared < autodiff::TVec > (initializer_list<double> {0, 0});
            passerPos = make_shared<geometry::CNPosition>();
        }
        else
        {
            for (int id : *r1)
            {
                passerPos = wm->robots.teammates.getTeamMatePosition(id);
                if (passerPos != nullptr)
                {
                    passer = make_shared < autodiff::TVec > (initializer_list<double> {passerPos->x, passerPos->y});
                }
                else
                {
                    cout << "Attackplay: No Passer Position in WM" << endl;
                    passerPos = make_shared<geometry::CNPosition>();
                    passer = make_shared < autodiff::TVec > (initializer_list<double> {0, 0});
                }
                break;
            }
        }

        //Get receiver position
        shared_ptr<vector<int>> r2 = rp->getAssignment()->getRobotsWorking(1434112675755);
        shared_ptr < autodiff::TVec > receiver = nullptr;
        if (r2->size() == 0 || r2 == nullptr)
        {
            cout << "Attackplay: No Receiver Assigned!?!?!?" << endl;
            receiver = make_shared < autodiff::TVec > (initializer_list<double> {0, 0});
        }
        else
        {
            for (int id : *r2)
            {
                shared_ptr < geometry::CNPosition > p = wm->robots.teammates.getTeamMatePosition(id);
                if (p != nullptr)
                {
                    receiver = make_shared < autodiff::TVec > (initializer_list<double> {p->x, p->y});
                }
                else
                {
                    cout << "Attackplay: No receiver Position in WM" << endl;
                    receiver = make_shared < autodiff::TVec > (initializer_list<double> {0, 0});
                }
                break;
            }
        }

        //Can this really happen?
        if (c->getStaticVars() == nullptr || c->getStaticVars()->size() == 0)
        {
            cout << "Attackplay: This error should never occure" << endl;
            c->setConstraint(dynamic_pointer_cast < alica::SolverTerm > (constraint));
            c->setUtility(dynamic_pointer_cast < alica::SolverTerm > (util));
            return;
        }
        vector < shared_ptr < autodiff::Term >> targetPosVec;
        targetPosVec.push_back(dynamic_pointer_cast < autodiff::Term > (c->getStaticVars()->at(0)));
        targetPosVec.push_back(dynamic_pointer_cast < autodiff::Term > (c->getStaticVars()->at(1)));
        shared_ptr < autodiff::TVec > target = make_shared < autodiff::TVec > (targetPosVec);

        //Pass should be at most 6m
        constraint = (ConstraintBuilder::distanceSqr(target, receiver) < autodiff::TermBuilder::constant(6000 * 6000));
        //Pass should be at most 2m away from the ball (relevant if we do not have the ball)
        shared_ptr < geometry::CNPoint2D > ballPos = wm->ball.getAlloBallPosition();
        if (ballPos != nullptr
                && (wm->whiteBoard.getPassMsg() == nullptr || wm->whiteBoard.getPassMsg()->receiverID != wm->getOwnId()))
        {
            shared_ptr < TVec > tvecBallPose = make_shared < TVec > (initializer_list<double> {ballPos->x, ballPos->y});
            constraint = constraint
                    & (ConstraintBuilder::distanceSqr(target, tvecBallPose)
                            > autodiff::TermBuilder::constant(1500 * 1500));
        }

        //Target position should be in the opponent half and in front of the passing robot
        //i.e. we avoid backward passes!
        if (passerPos->x <= 150)
        {
            constraint = constraint & target->getX() > autodiff::TermBuilder::constant(passerPos->x + 1000);
        }
        else
        {
            constraint = constraint & target->getX() > autodiff::TermBuilder::constant(500);
        }

        //Dont pass into penalty areas
        constraint = constraint & msl::MSLConstraintBuilder::outsideArea(msl::Areas::OppPenaltyArea, target);
        constraint = constraint & msl::MSLConstraintBuilder::outsideArea(msl::Areas::OwnPenaltyArea, target);

        //Pass at least 1.5 away from distance
        shared_ptr < vector<shared_ptr<geometry::CNPoint2D>>> opps = wm->robots.opponents.getOpponentsAlloClustered();
        vector < shared_ptr < autodiff::TVec >> oppVecs;

        shared_ptr < autodiff::Term > minOppDist = autodiff::TermBuilder::constant(1000000);

        if (opps == nullptr || opps->size() == 0)
        {
            minOppDist = autodiff::TermBuilder::constant(0);
        }
        else
        {
            for (int i = 0; i < opps->size(); i++)
            {
                oppVecs.push_back(
                        make_shared < autodiff::TVec > (initializer_list<double> {opps->at(i)->x, opps->at(i)->y}));
                constraint = constraint
                        & ConstraintBuilder::distanceSqr(target, oppVecs[i])
                                > autodiff::TermBuilder::constant(1500 * 1500); //get away from opps

                minOppDist = make_shared < autodiff::Min
                        > (minOppDist, ConstraintBuilder::distance(target, oppVecs.at(i)));

            }
        }

        //avoid teammates
        auto mates = wm->robots.teammates.getTeammatesAlloClustered();
        vector < shared_ptr < autodiff::TVec >> mateVec;
        if (mates != nullptr)
        {
            for (int i = 0; i < mates->size(); i++)
            {
                mateVec.push_back(
                        make_shared < autodiff::TVec > (initializer_list<double> {mates->at(i)->x, mates->at(i)->y}));
                constraint = constraint
                        & ConstraintBuilder::distanceSqr(target, mateVec[i])
                                > autodiff::TermBuilder::constant(2000 * 2000); //get away from mates
            }
        }

        double targetDist = abs(passerPos->y) + ff->FieldWidth * 0.5;
        targetDist += 500;

        //optimize distance to opps
        util = util + 100000
                - (make_shared < autodiff::Abs > (targetDist - ConstraintBuilder::distance(target, passer)))
                + minOppDist;
        c->setConstraint(dynamic_pointer_cast < alica::SolverTerm > (constraint));
        c->setUtility(dynamic_pointer_cast < alica::SolverTerm > (util));
        /*PROTECTED REGION END*/
    }

// State: Attack

// State: Attack

// State: MidFieldDefense

// State: MidFieldDefense

// State: Defend

// State: Defend

// State: Release

// State: Release

// State: ApproachPass

// State: ApproachPass

// State: InterceptPass

// State: InterceptPass

// State: InterceptPass

// State: InterceptPass

// State: ApproachPass

// State: ApproachPass

}
