/*
 * Game.cpp
 *
 *  Created on: Feb 24, 2015
 *      Author: Stefan Jakob
 */

#include <Game.h>
#include "MSLWorldModel.h"

namespace msl
{

	Game::Game(MSLWorldModel* wm)
	{
		this->wm = wm;
		this->gameState = GameState::NobodyInBallPossession;
		this->lastActiveSituation = Situation::Undefined;
		this->timeSinceStart = 0;
		this->mayScore = false;
		this->teamMateWithBall = 0;
		ownGoal = 0;
		oppGoal = 0;
		gameTime = 0;
		lastSituation = Situation::Undefined;
		currentSituation = Situation::Undefined;
		spinner = new ros::AsyncSpinner(4);
		spinner->start();
		refereeBoxInfoBodySub = n.subscribe("/RefereeBoxInfoBody", 10, &Game::onRefereeBoxInfoBody, (Game*)this);
		sc = SystemConfig::getInstance();
		ownTeamColor = (*this->sc)["Globals"]->get<string>("Globals", "OwnTeamColour", NULL);
		ownGoalColor = (*this->sc)["Globals"]->get<string>("Globals", "OwnGoalColour", NULL);
	}

	Game::~Game()
	{
		// TODO Auto-generated destructor stub
	}

	//TODO handle all situations
	void Game::onRefereeBoxInfoBody(msl_msgs::RefereeBoxInfoBodyPtr msg)
	{
		lock_guard<mutex> lock(refereeMutex);
		gameTime = msg->elapsedSeconds;
		if (currentSituation != Situation::Start)
		{
			lastSituation = currentSituation;
		}
		if (refereeBoxInfoBodyCommandData.size() > wm->getRingBufferLength())
		{
			refereeBoxInfoBodyCommandData.pop_back();

		}
		refereeBoxInfoBodyCommandData.push_front(msg);

		if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::start)
		{
			if (currentSituation != Situation::Start)
			{
				timeSinceStart = wm->getTime();
			}
			currentSituation = Situation::Start;
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::stop)
		{
			currentSituation = Situation::Stop;
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::command_joystick)
		{
			currentSituation = Situation::Joystick;
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::cancel)
		{
			currentSituation = Situation::Cancel;
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::droppedBall)
		{
			currentSituation = Situation::DroppedBall;
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::endGame)
		{
			currentSituation = Situation::EndGame;
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::cornerCyan)
		{
			if (ownTeamColor.compare("cyan"))
			{
				currentSituation = Situation::OwnCorner;
			}
			else
			{
				currentSituation = Situation::OppCorner;
			}
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::cornerMagenta)
		{
			if (ownTeamColor.compare("magenta"))
			{
				currentSituation = Situation::OwnCorner;
			}
			else
			{
				currentSituation = Situation::OppCorner;
			}
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::firstHalf)
		{
			currentSituation = Situation::FirstHalf;
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::freekickCyan)
		{
			if (ownTeamColor.compare("cyan"))
			{
				currentSituation = Situation::OwnFreekick;
			}
			else
			{
				currentSituation = Situation::OppFreekick;
			}
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::freekickMagenta)
		{
			if (ownTeamColor.compare("magenta"))
			{
				currentSituation = Situation::OwnFreekick;
			}
			else
			{
				currentSituation = Situation::OppFreekick;
			}
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::goalCyan)
		{
			lock_guard<mutex> lock(goalMutex);
			if (ownTeamColor.compare("cyan"))
			{
				ownGoal++;
			}
			else
			{
				oppGoal++;
			}
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::goalMagenta)
		{
			lock_guard<mutex> lock(goalMutex);
			if (ownTeamColor.compare("magenta"))
			{
				ownGoal++;
			}
			else
			{
				oppGoal++;
			}
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::goalkickCyan)
		{
			if (ownTeamColor.compare("cyan"))
			{
				currentSituation = Situation::OwnGoalkick;
			}
			else
			{
				currentSituation = Situation::OppGoalkick;
			}
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::goalkickMagenta)
		{
			if (ownTeamColor.compare("magenta"))
			{
				currentSituation = Situation::OwnGoalkick;
			}
			else
			{
				currentSituation = Situation::OppGoalkick;
			}
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::halftime)
		{
			currentSituation = Situation::HalfTime;
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::halt)
		{
			currentSituation = Situation::Halt;
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::kickoffCyan)
		{
			if (ownTeamColor.compare("cyan"))
			{
				currentSituation = Situation::OwnKickoff;
			}
			else
			{
				currentSituation = Situation::OppKickoff;
			}
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::kickoffMagenta)
		{
			if (ownTeamColor.compare("magenta"))
			{
				currentSituation = Situation::OwnKickoff;
			}
			else
			{
				currentSituation = Situation::OppKickoff;
			}
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::park)
		{
			currentSituation = Situation::Parking;
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::penaltyCyan)
		{
			if (ownTeamColor.compare("cyan"))
			{
				currentSituation = Situation::OwnPenalty;
			}
			else
			{
				currentSituation = Situation::OppPenalty;
			}
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::penaltyMagenta)
		{
			if (ownTeamColor.compare("magenta"))
			{
				currentSituation = Situation::OwnPenalty;
			}
			else
			{
				currentSituation = Situation::OppPenalty;
			}
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::ready)
		{
			currentSituation = Situation::Ready;
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::restart)
		{
			currentSituation = Situation::Restart;
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::secondHalf)
		{
			currentSituation = Situation::SecondHalf;
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::throwinCyan)
		{
			if (ownTeamColor.compare("cyan"))
			{
				currentSituation = Situation::OwnThrowin;
			}
			else
			{
				currentSituation = Situation::OppThrowin;
			}
		}
		else if ((int)msg->lastCommand == msl_msgs::RefereeBoxInfoBody::throwinMagenta)
		{
			if (ownTeamColor.compare("magenta"))
			{
				currentSituation = Situation::OwnThrowin;
			}
			else
			{
				currentSituation = Situation::OppThrowin;
			}
		}
		if ((int)msg->lastCommand != msl_msgs::RefereeBoxInfoBody::start
				&& (int)msg->lastCommand != msl_msgs::RefereeBoxInfoBody::stop)
		{
			lastActiveSituation = currentSituation;
		}
	}

	//time in nanoseconds
	unsigned long Game::getTimeSinceStart()
	{
		return timeSinceStart;
	}
	msl_msgs::RefereeBoxInfoBodyPtr Game::getRefereeBoxInfoBody()
	{
		lock_guard<mutex> lock(refereeMutex);
		if (refereeBoxInfoBodyCommandData.size() == 0)
		{
			return nullptr;
		}
		return refereeBoxInfoBodyCommandData.front();
	}

	bool Game::checkSituation(Situation situation)
	{
		lock_guard<mutex> lock(situationChecker);
		return currentSituation == situation;
	}

	Situation Game::getCurrentSituation()
	{
		return currentSituation;
	}

	Situation Game::getLastSituation()
	{
		return lastSituation;
	}

	int Game::getOppGoal()
	{
		return oppGoal;
	}

	int Game::getOwnGoal()
	{
		return ownGoal;
	}

	GameState Game::getGameState()
	{
		return gameState;
	}

	void Game::setGameState(GameState gameState)
	{
		this->gameState = gameState;
	}

	void Game::updateGameState()
	{
		// Find robot closest to ball
		auto robots = this->wm->robots.getPositionsOfTeamMates();
		shared_ptr<pair<int, shared_ptr<geometry::CNPosition>>> closestRobot = nullptr;
		double minDist = numeric_limits<double>::max();
		auto sharedBallPosition = wm->ball.getSharedBallPosition();
		if (sharedBallPosition == nullptr)
		{
			return;
		}
		bool ballPossession = false;
		for (shared_ptr<pair<int, shared_ptr<geometry::CNPosition>>> shwmData : *robots)
		{
			double currDist = shwmData->second->distanceTo(sharedBallPosition);
			if(closestRobot == nullptr || currDist < minDist)
			{	closestRobot = shwmData;
				minDist = currDist;
			}
			ballPossession |= *(wm->ball.getTeamMateBallPossession(shwmData->first));
		}

		if (closestRobot == nullptr)
		{
			return;
		}
		bool oppBallPossession = *(wm->ball.getOppBallPossession());
		GameState gs = getGameState();

		if (gs != GameState::Duel && ballPossession && oppBallPossession)
		{
			//cout << "State changed: Melee state" << endl;
			gs = GameState::Duel;
			this->teamMateWithBall = 0;
		}
		else if (gs != GameState::OwnBallPossession && ballPossession && !oppBallPossession)
		{
			//cout << "State changed: Attack state" << endl;
			gs = GameState::OwnBallPossession;
			setMayScore();
		}
		else if (gs != GameState::OppBallPossession && !ballPossession && oppBallPossession)
		{
			//cout << "State changed: Defend state" << endl;
			gs = GameState::OppBallPossession;
			this->teamMateWithBall = 0;
		}
		else if (gs != GameState::NobodyInBallPossession && !ballPossession && !oppBallPossession)
		{
			//cout << "State changed: Conflict state" << endl;
			gs = GameState::NobodyInBallPossession;
		}

		setGameState(gs);
	}

	bool Game::isMayScore()
	{
		return mayScore;
	}

	void Game::setMayScore()
	{
		shared_ptr<geometry::CNPosition> capturePos = nullptr;
		int teamMateWithBallNow = 0;
		for (shared_ptr<pair<int, shared_ptr<geometry::CNPosition>>> shwmData : *(this->wm->robots.getPositionsOfTeamMates()))
		{
			if (*(wm->ball.getTeamMateBallPossession(shwmData->first)))
			{
				capturePos = shwmData->second;
				teamMateWithBallNow = shwmData->first;
			}
		}

		if (capturePos == nullptr || capturePos->x < 0)
			mayScore = false;

		if (capturePos->x > 50 && teamMateWithBall != teamMateWithBallNow)
		{
			mayScore = true;
		}

		teamMateWithBall = teamMateWithBallNow;
	}

} /* namespace alica */

