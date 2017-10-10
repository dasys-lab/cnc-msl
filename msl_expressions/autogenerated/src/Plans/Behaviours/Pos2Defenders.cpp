using namespace std;
#include "Plans/Behaviours/Pos2Defenders.h"

/*PROTECTED REGION ID(inccpp1444834678756) ENABLED START*/ //Add additional includes here
#include <limits>
using geometry::CNPointAllo;
using geometry::CNVecAllo;
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1444834678756) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    Pos2Defenders::Pos2Defenders() :
            DomainBehaviour("Pos2Defenders")
    {
        /*PROTECTED REGION ID(con1444834678756) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    Pos2Defenders::~Pos2Defenders()
    {
        /*PROTECTED REGION ID(dcon1444834678756) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void Pos2Defenders::run(void* msg)
    {
        /*PROTECTED REGION ID(run1444834678756) ENABLED START*/ //Add additional options here
        msl::RobotMovement rm;
        auto alloBallPos = wm->ball->getPositionAllo();

        if (!alloBallPos)
        {
            //assume ball is in the middle (this is at least true for the beginning phase of kick off standard)
            alloBallPos = CNPointAllo(0, 0);
        }

        nonstd::optional<std::vector<geometry::CNPointAllo>> additionalPoints;
        // add alloBall to path planning
        additionalPoints->push_back(*alloBallPos);

        this->keeperPos = wm->robots->teammates.getTeammatePositionBuffer(keeperId).getLastValidContent();
        int ownId = this->wm->getOwnId();
        auto ownEp = this->getRunningPlan()->getParent().lock()->getAssignment()->getEntryPointOfRobot(ownId);
        auto robotsInOwnEp = this->getRunningPlan()->getParent().lock()->getAssignment()->getRobotsWorking(ownEp);
        auto firstDefPos = (*alloBallPos) + CNVecAllo(-2500, -600);
        auto secondDefPos = (*alloBallPos) + CNVecAllo(-4000, 2300);
        auto firstDef = wm->robots->teammates.getTeammatePositionBuffer((*robotsInOwnEp)[0]).getLastValidContent();
        msl_actuator_msgs::MotionControl mc;

        if(!firstDef) {
            return;
        }
        if (robotsInOwnEp->size() == 2)
        {

            auto secondDef = wm->robots->teammates.getTeammatePositionBuffer((*robotsInOwnEp)[1]).getLastValidContent();
            if(!secondDef) {
                return;
            }
            //first Defender is closer to first position
            if ((firstDef->distanceTo(firstDefPos)) < secondDef->distanceTo(firstDefPos))
            {
                //i am first Defender
                if (ownId == (*robotsInOwnEp)[0])
                {

//                    mc = msl::RobotMovement::moveToPointCarefully(firstDefPos->alloToEgo(*firstDef),
//                                                                  alloBallPos->alloToEgo(*firstDef), 0,
//                                                                  additionalPoints);
                    query.egoDestinationPoint = firstDefPos.toEgo(*firstDef);
                    query.egoAlignPoint = alloBallPos->toEgo(*firstDef);
                    query.additionalPoints = additionalPoints;
                    mc = rm.moveToPoint(query);

                }
                else
                {
//                    mc = msl::RobotMovement::moveToPointCarefully(secondDefPos->alloToEgo(*secondDef),
//                                                                  alloBallPos->alloToEgo(*secondDef), 0,
//                                                                  additionalPoints);
                    query.egoDestinationPoint = secondDefPos.toEgo(*secondDef);
                    query.egoAlignPoint = alloBallPos->toEgo(*secondDef);
                    query.additionalPoints = additionalPoints;
                    mc = rm.moveToPoint(query);
                }

            }
            //second Defender is closer to first position
            else
            {
                //i am second Defender
                if (ownId == (*robotsInOwnEp)[0])
                {

//                    mc = msl::RobotMovement::moveToPointCarefully(secondDefPos->alloToEgo(*firstDef),
//                                                                  alloBallPos->alloToEgo(*firstDef), 0);
                    query.egoDestinationPoint = secondDefPos.toEgo(*firstDef);
                    query.egoAlignPoint = alloBallPos->toEgo(*firstDef);
                    mc = rm.moveToPoint(query);

                }
                else
                {
//                    mc = msl::RobotMovement::moveToPointCarefully(firstDefPos->alloToEgo(*secondDef),
//                                                                  alloBallPos->alloToEgo(*secondDef), 0);
                    query.egoDestinationPoint = firstDefPos.toEgo(*secondDef);
                    query.egoAlignPoint = alloBallPos->toEgo(*secondDef);
                    mc = rm.moveToPoint(query);
                }

            }

        }
        else
        {
//            mc = msl::RobotMovement::moveToPointCarefully(firstDefPos->alloToEgo(*firstDef),
//                                                          alloBallPos->alloToEgo(*firstDef), 0);
            query.egoDestinationPoint = firstDefPos.toEgo(*firstDef);
            query.egoAlignPoint = alloBallPos->toEgo(*firstDef);
            mc = rm.moveToPoint(query);
        }

        if (!std::isnan(mc.motion.translation))
        {
            send(mc);
        }
        else
        {
            cout << "Motion command is NaN!" << endl;
        }
        /*PROTECTED REGION END*/
    }
    void Pos2Defenders::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1444834678756) ENABLED START*/ //Add additional options here
        // get the teammate which is the closest to the own goal mid (hopefully it is the keeper)
        auto positions = this->wm->robots->teammates.getPositionsOfTeamMates();
        auto ownGoalMid = wm->field->posOwnGoalMid();
        double smallestDist = std::numeric_limits<double>::max();
        for (auto pos : *positions)
        {
            /* check is obsolete as only existing positions are returned
             if (pos.second != nullptr)*/

            double tmpDist = pos.second.distanceTo(ownGoalMid);
            if (tmpDist < smallestDist)
            {
                smallestDist = tmpDist;
                this->keeperId = pos.first;
                this->keeperPos = nonstd::make_optional<geometry::CNPositionAllo> (pos.second);
            }
        }

        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1444834678756) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
