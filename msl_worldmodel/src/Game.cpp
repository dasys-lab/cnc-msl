#include "Game.h"
#include "Ball.h"
#include "MSLWorldModel.h"
#include "Robots.h"
#include <SystemConfig.h>

using nonstd::optional;
using nonstd::nullopt;
using supplementary::InformationElement;
using supplementary::InfoBuffer;

namespace msl
{

Game::Game(MSLWorldModel *wm, int ringBufferLength)
    : refBoxCommandBuffer(ringBufferLength)
{
    this->wm = wm;
    this->gameState = GameState::NobodyInBallPossession;
    this->timeSinceStart = 0;
    this->passReceived = false;
    this->mayScore = false;
    this->teamMateWithBall = 0;
    ownGoal = 0;
    oppGoal = 0;
    gameTime = 0;
    situation = Situation::Undefined;
    spinner = new ros::AsyncSpinner(4);
    spinner->start();
    refBoxCommandSub = n.subscribe("/RefereeBoxInfoBody", 10, &Game::onRefBoxCommand, (Game *)this);
    robotCommandSub = n.subscribe("/RobotCommand", 10, &Game::onRobotCommand, (Game *)this);

    // Set own Team Color
    this->sc = supplementary::SystemConfig::getInstance();
    string tmpOwnTeamColor = (*this->sc)["Globals"]->get<string>("Globals", "OwnTeamColour", NULL);
    if (tmpOwnTeamColor.compare("cyan") == 0)
    {
        cout << "MSL-WM::Game: Own team color is CYAN" << endl;
        ownTeamColor = Color::Cyan;
    }
    else if (tmpOwnTeamColor.compare("magenta") == 0)
    {
        ownTeamColor = Color::Magenta;
        cout << "MSL-WM::Game: Own team color is MAGENTA" << endl;
    }
    else
    {
        ownTeamColor = Color::UnknownColor;
        cerr << "MSL-WM::Game: Own team color is UNKNOWN!" << endl;
    }

    // Set own Goal Color
    string tmpeOwnGoalColor = (*this->sc)["Globals"]->get<string>("Globals", "OwnGoalColour", NULL);
    if (tmpeOwnGoalColor.find("b") != tmpeOwnGoalColor.npos)
    {
        ownGoalColor = Color::Blue;
        cout << "MSL-WM::Game: OwnGoal is BLUE" << endl;
    }
    else if (tmpeOwnGoalColor.find("y") != tmpeOwnGoalColor.npos)
    {
        ownGoalColor = Color::Yellow;
        cout << "MSL-WM::Game: OwnGoal is YELLOW" << endl;
    }
    else
    {
        cerr << "MSL-WM::Game: Own goal color is UNKNOWN!" << endl;
        exit(0);
        ownGoalColor = Color::UnknownColor;
    }
}

Game::~Game()
{
}

void Game::onRobotCommand(robot_control::RobotCommandPtr msg)
{
    if (msg->receiverId != 0 && msg->receiverId != wm->getOwnId())
    {
        return;
    }

    lock_guard<mutex> lock(refereeMutex);
    switch (msg->cmd)
    {
    case robot_control::RobotCommand::START:
        situation = Situation::Start;
        break;
    case robot_control::RobotCommand::STOP:
        situation = Situation::Stop;
        break;
    }
}

void Game::onRefBoxCommand(msl_msgs::RefBoxCommandConstPtr msg)
{
    // Put the referee box command into the ringbuffer

    auto refBoxCmd = std::make_shared<InformationElement<msl_msgs::RefBoxCommand>>(*msg, wm->getTime(), this->maxValidity, 1);
    refBoxCommandBuffer.add(refBoxCmd);
    // Set the current refbox situation
    lock_guard<mutex> lock(refereeMutex);
    switch (msg->cmd)
    {
    case msl_msgs::RefBoxCommand::START:
        if (situation != Situation::Start)
        {
            timeSinceStart = wm->getTime();
        }
        situation = Situation::Start;
        this->teamMateWithBall = 0;
        this->passReceived = false;
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
    { // scope for mutex
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
    default:
        cerr << "MSLWM-Game: Unknown Referee Box Cmd received!" << endl;
    }

    gameTime = msg->elapsedSeconds;
}

unsigned long Game::getTimeSinceStart()
{
    return timeSinceStart;
}

const InfoBuffer<msl_msgs::RefBoxCommand> &Game::getRefBoxCommandBuffer() const
{
	return this->refBoxCommandBuffer;
}


bool Game::checkSituation(Situation situation)
{
    lock_guard<mutex> lock(situationChecker);
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
    auto robots = this->wm->robots->teammates.getPositionsOfTeamMates();
    shared_ptr<pair<int, geometry::CNPositionAllo>> closestRobot;
    double minDist = numeric_limits<double>::max();
    auto sharedBallPosition = wm->ball->getAlloSharedBallPositionBuffer().getLastValidContent();
    if (!sharedBallPosition)
    {
        return;
    }
    bool ballPossession = false;
    for (pair<int, geometry::CNPositionAllo> shwmData : *robots)
    {
        double currDist = shwmData.second.distanceTo(*sharedBallPosition);
        if (closestRobot == nullptr || currDist < minDist)
        {
            closestRobot = make_shared<pair<int, geometry::CNPositionAllo>>(shwmData);
            minDist = currDist;
        }

        auto realShwm = wm->robots->getSHWMDataBuffer(shwmData.first)->getLastValid();

        // auto hasball = (  ball.getTeamMateBallPossession(shwmData->first));
        if (realShwm != nullptr)
        {
            ballPossession |= realShwm->getInformation().ballInPossession;
        }
    }

    if (closestRobot == nullptr)
    {
        cout << "Game::updateGameState(): closestRobot == nullptr -> is this even possible?" << endl;
        return;
    }

    auto oppPoss = wm->ball->getOppBallPossession();
    bool oppBallPossession = false;
    if (oppPoss != nullopt)
    {
        oppBallPossession = oppPoss.value();
    }

    GameState gs = getGameState();

    if (gs != GameState::Duel && ballPossession && oppBallPossession)
    {
        cout << "Game::updateGameState(): State changed: Duel state" << endl;
        gs = GameState::Duel;
        this->teamMateWithBall = 0;
//        passReceived = false;
    }
    else if (gs != GameState::OwnBallPossession && ballPossession && !oppBallPossession)
    {
        cout << "Game::updateGameState(): State changed: OwnBallPossession state" << endl;
        gs = GameState::OwnBallPossession;
        setMayScore();
    }
    else if (gs != GameState::OppBallPossession && !ballPossession && oppBallPossession)
    {
        cout << "Game::updateGameState(): State changed: OppBallPossession state" << endl;
        gs = GameState::OppBallPossession;
        this->teamMateWithBall = 0;
        passReceived = false;
    }
    else if (gs != GameState::NobodyInBallPossession && !ballPossession && !oppBallPossession)
    {
        cout << "Game::updateGameState(): State changed: NobodyInBallPossession state" << endl;
        gs = GameState::NobodyInBallPossession;
        passReceived = false;
    }

    setGameState(gs);
}

bool Game::isMayScore()
{
    return mayScore;
}

void Game::setMayScore()
{
    optional<geometry::CNPositionAllo> capturePos = nullopt;
    int teamMateWithBallNow = 0;
    auto robotPoses = this->wm->robots->teammates.getPositionsOfTeamMates();
    if (robotPoses != nullptr)
    {
        for (pair<int, geometry::CNPositionAllo> shwmData : *robotPoses)
        {
            auto teamMateBallPosession = wm->ball->getTeamMateBallPossession(shwmData.first);
            if (teamMateBallPosession && *teamMateBallPosession)
            {
                capturePos = shwmData.second;
                teamMateWithBallNow = shwmData.first;
            }
        }
    }

    // update to rule changes 2017: goal valid after dribbling to opp side after pass; not valid after winning duel without passing

    if (!capturePos || capturePos->x < 0 || !passReceived)
    {
        mayScore = false;
    }
    if (teamMateWithBall != 0 && teamMateWithBallNow != 0 && teamMateWithBall != teamMateWithBallNow)
    {
        passReceived = true;
    }

    teamMateWithBall = teamMateWithBallNow;

    auto teamMatePos = wm->robots->teammates.getTeammatePositionBuffer(teamMateWithBall).getLastValid();
    if (teamMatePos != nullptr && teamMatePos->getInformation().x > 50 && passReceived)
    {
        cout << "tmwb : " << teamMateWithBall << "tmwbn: " << teamMateWithBallNow
             << "=========================================GAME: SETTING MAYSCORE =============================================================" << endl;
        mayScore = true;
    }
}

} /* namespace alica */
