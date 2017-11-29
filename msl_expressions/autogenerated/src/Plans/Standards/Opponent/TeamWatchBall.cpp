#include "Plans/Standards/Opponent/TeamWatchBall.h"

/*PROTECTED REGION ID(inccpp1457015532224) ENABLED START*/ //Add additional includes here
#include <Rules.h>
#include <Ball.h>
#include <WhiteBoard.h>
#include <MSLWorldModel.h>
using nonstd::nullopt;
using nonstd::optional;
using std::cout;
using std::endl;
/*PROTECTED REGION END*/
namespace alica
{
    /*PROTECTED REGION ID(staticVars1457015532224) ENABLED START*/ //initialise static variables here
    /*PROTECTED REGION END*/
    TeamWatchBall::TeamWatchBall() :
            DomainBehaviour("TeamWatchBall")
    {
        /*PROTECTED REGION ID(con1457015532224) ENABLED START*/ //Add additional options here
        maxSend = 4;
        moveDistance = 2000;
        ballPos = nullopt;

        ballMovedOccurrences = 0;
        maxBallMovedOccurrences = 3;
        itcounter = 0;

        moveDistance = (*this->sc)["StandardSituation"]->get<double>("WatchBall.StandardBallMovedDistance", NULL);
        /*PROTECTED REGION END*/
    }
    TeamWatchBall::~TeamWatchBall()
    {
        /*PROTECTED REGION ID(dcon1457015532224) ENABLED START*/ //Add additional options here
        /*PROTECTED REGION END*/
    }
    void TeamWatchBall::run(void* msg)
    {
        /*PROTECTED REGION ID(run1457015532224) ENABLED START*/ //Add additional options here
        if (wm->whiteBoard->getWatchBallMsgBuffer().getLastValidContent())
        {
            this->setSuccess(true);
            return;
        }

        itcounter++;
        if (itcounter < 10)
            return;

        auto egoBall = wm->ball->getPositionEgo();

        if (!egoBall)
        {
            return;
        }

        optional<geometry::CNPointEgo> posV = nullopt;
        if (ballPos)
        {

            auto b1 = wm->ball->getVisionBallPositionBuffer().getLast(0);
            if (b1)
            {
                auto pos1 = b1->getInformation();
                if (pos1.length() < 6300)
                {
                    posV = nonstd::make_optional<geometry::CNPointEgo>(pos1);
                }
            }

            auto b2 = wm->ball->getVisionBallPositionBuffer().getLast(1);
            if (b2)
            {
                auto pos2 = b2->getInformation();
                if (!posV)
                {
                    if (pos2.length() < 6300)
                    {
                        posV = nonstd::make_optional<geometry::CNPointEgo>(pos2);
                    }
                }
                else if (pos2.length() < 6300)
                {
                    posV = geometry::CNPointEgo(posV->x + pos2.x, posV->y + pos2.y);
                    posV = geometry::CNPointEgo(posV->x / 2, posV->y / 2);
                }
            }
            /*auto b1 = wm->ball->getVisionBallPosition(0);
             if (b1 != nullptr && b1->length() < 6300)
             posV = b1->clone();
             auto b2 = wm->ball->getVisionBallPosition(1);
             if (posV == nullptr && b2 != nullptr && b2->length() < 6300)
             {
             posV = b2->clone();
             }
             else if (b2 != nullptr)
             {
             posV = posV + b2->clone();
             posV = posV / 2.0;
             }

             if (wm->ball->getVisionBallPositionAndCertaincy()->second < 0.5)
             posV = nullptr;
             if (ballPos != nullptr && ballPos->length() > 4000)
             posV = nullptr;
             if (posV != nullptr)
             posV = posV - ballPos;*/

        }
        else
        {
            this->initialiseParameters();
        }
        if ((posV && posV->length() > this->moveDistance))
        {
            ballMovedOccurrences++;
        }
        else
        {
            ballMovedOccurrences = 0;
        }
        //if((WM.EgoBallPosition.Distance() < 4000 &&WorldHelper.BallMoved(WM)) || (ballMovedOccurrences >= maxBallMovedOccurrences))
        if ((egoBall->length() < 6000 && wm->ball->ballMovedSignificantly())
                || (ballMovedOccurrences >= maxBallMovedOccurrences))
        {
            //send messages to other whiteboards
            msl_helper_msgs::WatchBallMsg msg;

            msg.validFor = 200000000; //200msec

            for (int i = 0; i < maxSend; i++)
            {
                send(msg);
            }

            this->setSuccess(true);

//        shared_ptr < geometry::CNPoint2D > posV = nullptr;
//        if (ballPos != nullptr)
//        {
//            auto b1 = wm->ball->getVisionBallPosition(0);
//            if (b1 != nullptr && b1->length() < 6300)
//                posV = b1->clone();
//            auto b2 = wm->ball->getVisionBallPosition(1);
//            if (posV == nullptr && b2 != nullptr && b2->length() < 6300)
//            {
//                posV = b2->clone();
//            }
//            else if (b2 != nullptr)
//            {
//                posV = posV + b2->clone();
//                posV = posV / 2.0;
//            }
//
//            if (wm->ball->getVisionBallPositionAndCertaincy()->second < 0.5)
//                posV = nullptr;
//            if (ballPos != nullptr && ballPos->length() > 4000)
//                posV = nullptr;
//            if (posV != nullptr)
//                posV = posV - ballPos;
//        }
//        else
//        {
//            this->initialiseParameters();
//        }
//        if ((posV != nullptr && posV->length() > this->moveDistance))
//        {
//            ballMovedOccurrences++;
//        }
//        else
//        {
//            ballMovedOccurrences = 0;
//        }
//        //if((WM.EgoBallPosition.Distance() < 4000 &&WorldHelper.BallMoved(WM)) || (ballMovedOccurrences >= maxBallMovedOccurrences))
//        if ((egoBall->length() < 6000 && wm->ball->ballMovedSignificantly())
//                || (ballMovedOccurrences >= maxBallMovedOccurrences))
//        {
//            //send messages to other whiteboards
//            msl_helper_msgs::WatchBallMsg msg;
//
//            msg.validFor = 200000000; //200msec
//
//            for (int i = 0; i < maxSend; i++)
//            {
//                send(msg);
//            }
//
//            this->setSuccess(true);
        }
        /*PROTECTED REGION END*/
    }
    void TeamWatchBall::initialiseParameters()
    {
        /*PROTECTED REGION ID(initialiseParameters1457015532224) ENABLED START*/ //Add additional options here
        ballPos = nullopt;
        ballMovedOccurrences = 0;
        /*
         auto b1 = wm->ball->getVisionBallPosition(0);
         if (b1 != nullptr && b1->length() < 6300)
         ballPos = b1->clone();
         */

        auto b1 = wm->ball->getVisionBallPositionBuffer().getLast(0);
        if (b1)
        {
            auto pos1 = b1->getInformation();
            if (pos1.length() < 6300)
            {
                this->ballPos = nonstd::make_optional<geometry::CNPointEgo>(pos1);
            }
        }

        auto b2 = wm->ball->getVisionBallPositionBuffer().getLast(1);
        if (b2)
        {
            auto pos2 = b2->getInformation();
            if (!ballPos)
            {
                if (pos2.length() < 6300)
                {
                    this->ballPos = nonstd::make_optional<geometry::CNPointEgo>(pos2);
                }
            }
            else if (pos2.length() < 6300)
            {
                ballPos = nonstd::make_optional<geometry::CNPointEgo>(ballPos->x + pos2.x, ballPos->y + pos2.y);
                ballPos = nonstd::make_optional<geometry::CNPointEgo>(ballPos->x / 2, ballPos->y / 2);
            }
        }
        /*
         auto b2 = wm->ball->getVisionBallPosition(1);
         if (ballPos == nullptr && b2 != nullptr && b2->length() < 6300)
         {
         ballPos = b2->clone();
         }
         else if (b2 != nullptr && b2->length() < 6300)
         {
         ballPos = ballPos + b2;
         ballPos = ballPos / 2.0;
         }
         */

        //TODO this method doesn't exist anymore :<
        if (wm->ball->getVisionBallPositionBuffer().getLast()->getCertainty() < 0.4)
            ballPos = nullopt;

        if (ballPos && ballPos->length() > (msl::Rules::getInstance()->getStayAwayRadius() + 750))
            ballPos = nullopt;

        itcounter = 0;
        /*PROTECTED REGION END*/
    }
/*PROTECTED REGION ID(methods1457015532224) ENABLED START*/ //Add additional methods here
/*PROTECTED REGION END*/
} /* namespace alica */
