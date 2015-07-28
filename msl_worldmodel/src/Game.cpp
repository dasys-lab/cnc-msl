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

	Game::Game(MSLWorldModel* wm, int ringBufferLength) :
			refBoxCommand(ringBufferLength)
	{
		this->wm = wm;
		this->gameState = GameState::NobodyInBallPossession;
		this->timeSinceStart = 0;
		this->mayScore = false;
		this->teamMateWithBall = 0;
		ownGoal = 0;
		oppGoal = 0;
		gameTime = 0;
		situation = Situation::Undefined;
		spinner = new ros::AsyncSpinner(4);
		spinner->start();
		refereeBoxInfoBodySub = n.subscribe("/RefereeBoxInfoBody", 10, &Game::onRefBoxCommand, (Game*)this);
		robotCommandSub = n.subscribe("/RobotCommand", 10, &Game::onRobotCommand, (Game*)this);

		// Set own Team Color
		sc = SystemConfig::getInstance();
		string tmpOwnTeamColor = (*this->sc)["Globals"]->get<string>("Globals", "OwnTeamColour", NULL);
		if (tmpOwnTeamColor.compare("cyan"))
		{
			ownTeamColor = Color::Cyan;
		}
		else if (tmpOwnTeamColor.compare("magenta"))
		{
			ownTeamColor = Color::Magenta;
		}
		else
		{
			cerr << "MSL-WM::Game: Own team color is unknown!" << endl;
			ownTeamColor = Color::UnknownColor;
		}

		// Set own Goal Color
		string tmpeOwnGoalColor = (*this->sc)["Globals"]->get<string>("Globals", "OwnGoalColour", NULL);
		if (tmpOwnTeamColor.compare("blue"))
		{
			ownGoalColor = Color::Blue;
		}
		else if (tmpOwnTeamColor.compare("yellow"))
		{
			ownGoalColor = Color::Yellow;
		}
		else
		{
			cerr << "MSL-WM::Game: Own goal color is unknown!" << endl;
			ownGoalColor = Color::UnknownColor;
		}
	}

	Game::~Game()
	{
		// TODO Auto-generated destructor stub
	}

	void Game::onRobotCommand(rqt_robot_control::RobotCommandPtr msg)
	{
		if (msg->receiverId != 0 && msg->receiverId != wm->getOwnId())
		{
			return;
		}

		lock_guard<mutex> lock(refereeMutex);
		switch (msg->cmd)
		{
			case rqt_robot_control::RobotCommand::START:
				situation = Situation::Start;
				break;
			case rqt_robot_control::RobotCommand::STOP:
				situation = Situation::Stop;
				break;
		}

	}

	//TODO handle all situations
	void Game::onRefBoxCommand(msl_msgs::RefBoxCommandPtr msg)
	{
		cout << "onRefBoxCommand" << endl;
		// Put the referee box command into the ringbuffer

		/*
		 * In order to convert the boost::shared_ptr to a std::shared_ptr
		 * we use the conversion suggested in this post:
		 * http://stackoverflow.com/questions/12314967/cohabitation-of-boostshared-ptr-and-stdshared-ptr
		 */
		shared_ptr<msl_msgs::RefBoxCommand> cmd = shared_ptr<msl_msgs::RefBoxCommand>(
				msg.get(), [msg](msl_msgs::RefBoxCommand*) mutable
				{	msg.reset();});
		cout << "onRefBoxCommand2" << endl;
		shared_ptr<InformationElement<msl_msgs::RefBoxCommand>> refBoxCmd = make_shared<
				InformationElement<msl_msgs::RefBoxCommand>>(cmd, wm->getTime());
		refBoxCmd->certainty = 1.0;
		refBoxCommand.add(refBoxCmd);
		cout << "onRefBoxCommand3" << endl;
		// Set the current refbox situation
		lock_guard<mutex> lock(refereeMutex);
		cout << "onRefBoxCommand4" << endl;
		switch (msg->cmd)
		{
			case msl_msgs::RefBoxCommand::START:
				if (situation != Situation::Start)
				{
					timeSinceStart = wm->getTime();
				}
				cout << "Situation: Start" << endl;
				situation = Situation::Start;
				break;
			case msl_msgs::RefBoxCommand::STOP:
				situation = Situation::Stop;
				break;
			case msl_msgs::RefBoxCommand::COMMAND_JOYSTICK:
				situation = Situation::Joystick;
				break;
			case msl_msgs::RefBoxCommand::CANCEL:
				situation = Situation::Cancel;
				break;
			case msl_msgs::RefBoxCommand::DROPBALL:
				situation = Situation::DropBall;
				break;
			case msl_msgs::RefBoxCommand::END_GAME:
				situation = Situation::EndGame;
				break;
			case msl_msgs::RefBoxCommand::FIRST_HALF:
				situation = Situation::FirstHalf;
				break;
			case msl_msgs::RefBoxCommand::HALFTIME:
				situation = Situation::HalfTime;
				break;
			case msl_msgs::RefBoxCommand::SECOND_HALF:
				situation = Situation::SecondHalf;
				break;
			case msl_msgs::RefBoxCommand::HALT:
				situation = Situation::Halt;
				break;
			case msl_msgs::RefBoxCommand::PARK:
				situation = Situation::Parking;
				break;
			case msl_msgs::RefBoxCommand::READY:
				situation = Situation::Ready;
				break;
			case msl_msgs::RefBoxCommand::RESTART:
				situation = Situation::Restart;
				break;
			case msl_msgs::RefBoxCommand::CORNER_CYAN:
				if (ownTeamColor == Color::Cyan)
				{
					situation = Situation::OwnCorner;
				}
				else if (ownTeamColor == Color::Magenta)
				{
					situation = Situation::OppCorner;
				}
				break;
			case msl_msgs::RefBoxCommand::CORNER_MAGENTA:
				if (ownTeamColor == Color::Magenta)
				{
					situation = Situation::OwnCorner;
				}
				else if (ownTeamColor == Color::Cyan)
				{
					situation = Situation::OppCorner;
				}
				break;
			case msl_msgs::RefBoxCommand::FREEKICK_CYAN:
				if (ownTeamColor == Color::Cyan)
				{
					situation = Situation::OwnFreekick;
				}
				else if (ownTeamColor == Color::Magenta)
				{
					situation = Situation::OppFreekick;
				}
				break;
			case msl_msgs::RefBoxCommand::FREEKICK_MAGENTA:
				if (ownTeamColor == Color::Magenta)
				{
					situation = Situation::OwnFreekick;
				}
				else if (ownTeamColor == Color::Cyan)
				{
					situation = Situation::OppFreekick;
				}
				break;
			case msl_msgs::RefBoxCommand::GOALKICK_CYAN:
				if (ownTeamColor == Color::Cyan)
				{
					situation = Situation::OwnGoalkick;
				}
				else if (ownTeamColor == Color::Magenta)
				{
					situation = Situation::OppGoalkick;
				}
				break;
			case msl_msgs::RefBoxCommand::GOALKICK_MAGENTA:
				if (ownTeamColor == Color::Magenta)
				{
					situation = Situation::OwnGoalkick;
				}
				else if (ownTeamColor == Color::Cyan)
				{
					situation = Situation::OppGoalkick;
				}
				break;
			case msl_msgs::RefBoxCommand::KICKOFF_CYAN:
				if (ownTeamColor == Color::Cyan)
				{
					situation = Situation::OwnKickoff;
				}
				else if (ownTeamColor == Color::Magenta)
				{
					situation = Situation::OppKickoff;
				}
				break;
			case msl_msgs::RefBoxCommand::KICKOFF_MAGENTA:
				if (ownTeamColor == Color::Magenta)
				{
					situation = Situation::OwnKickoff;
				}
				else if (ownTeamColor == Color::Cyan)
				{
					situation = Situation::OppKickoff;
				}
				break;
			case msl_msgs::RefBoxCommand::PENALTY_CYAN:
				if (ownTeamColor == Color::Cyan)
				{
					situation = Situation::OwnPenalty;
				}
				else if (ownTeamColor == Color::Magenta)
				{
					situation = Situation::OppPenalty;
				}
				break;
			case msl_msgs::RefBoxCommand::PENALTY_MAGENTA:
				if (ownTeamColor == Color::Magenta)
				{
					situation = Situation::OwnPenalty;
				}
				else if (ownTeamColor == Color::Cyan)
				{
					situation = Situation::OppPenalty;
				}
				break;
			case msl_msgs::RefBoxCommand::THROWIN_CYAN:
				if (ownTeamColor == Color::Cyan)
				{
					situation = Situation::OwnThrowin;
				}
				else if (ownTeamColor == Color::Magenta)
				{
					situation = Situation::OppThrowin;
				}
				break;
			case msl_msgs::RefBoxCommand::THROWIN_MAGENTA:
				if (ownTeamColor == Color::Magenta)
				{
					situation = Situation::OwnThrowin;
				}
				else if (ownTeamColor == Color::Cyan)
				{
					situation = Situation::OppThrowin;
				}
				break;
			case msl_msgs::RefBoxCommand::GOAL_CYAN:
			{ // scope for mutex
				lock_guard<mutex> lock(goalMutex);
				if (ownTeamColor == Color::Cyan)
				{
					ownGoal++;
				}
				else if (ownTeamColor == Color::Magenta)
				{
					oppGoal++;
				}
			}
				break;
			case msl_msgs::RefBoxCommand::GOAL_MAGENTA:
			{ //scope for mutex
				lock_guard<mutex> lock(goalMutex);
				if (ownTeamColor == Color::Magenta)
				{
					ownGoal++;
				}
				else if (ownTeamColor == Color::Cyan)
				{
					oppGoal++;
				}
			}
				break;
		}

		gameTime = msg->elapsedSeconds;
	}

	unsigned long Game::getTimeSinceStart()
	{
		return timeSinceStart;
	}

	shared_ptr<msl_msgs::RefBoxCommand> Game::getRefBoxCommand(int index)
	{
		lock_guard<mutex> lock(refereeMutex);
		auto x = refBoxCommand.getLast(index);
		if (x == nullptr)
		{
			return nullptr;
		}
		return x->getInformation();
	}

	bool Game::checkSituation(Situation situation)
	{
		lock_guard<mutex> lock(situationChecker);
		cout << "check: " << this->situation << "|" << situation << endl;
		return this->situation == situation;
	}

	Situation Game::getSituation()
	{
		return situation;
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

