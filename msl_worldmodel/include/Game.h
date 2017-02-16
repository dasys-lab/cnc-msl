#pragma once

#include "InfoBuffer.h"
#include "GameState.h"
#include "InformationElement.h"
#include "MSLEnums.h"

#include <msl_msgs/RefBoxCommand.h>
#include <robot_control/RobotCommand.h>
#include <ros/ros.h>

#include <mutex>
#include <memory>

namespace supplementary
{
class SystemConfig;
}

namespace msl
{

class MSLWorldModel;
class Game
{
  public:
    Game(MSLWorldModel *wm, int ringBufferLength);
    virtual ~Game();
    void onRobotCommand(robot_control::RobotCommandPtr msg);
    void onRefBoxCommand(msl_msgs::RefBoxCommandConstPtr msg);
    const InfoBuffer<msl_msgs::RefBoxCommand> &getRefBoxCommandBuffer() const;
    bool checkSituation(Situation situation);
    int getOppGoal();
    int getOwnGoal();
    unsigned long getTimeSinceStart();
    Situation getSituation();
    GameState getGameState();
    void setGameState(GameState gameState);
    void updateGameState();
    bool isMayScore();
    Color ownTeamColor;
    Color ownGoalColor;
    long gameTime;

  private:
    void setMayScore();

    MSLWorldModel *wm;
    Situation situation;
    ros::NodeHandle n;
    ros::AsyncSpinner *spinner;
    ros::Subscriber refBoxCommandSub;
    ros::Subscriber robotCommandSub;
    InfoBuffer<msl_msgs::RefBoxCommand> refBoxCommandBuffer;
    std::mutex refereeMutex;
    std::mutex situationChecker;
    std::mutex goalMutex;
    supplementary::SystemConfig *sc;
    int ownGoal;
    int oppGoal;
    unsigned long timeSinceStart;
    GameState gameState;
    int teamMateWithBall;
    bool passReceived;
    bool mayScore;
    const InfoTime maxValidity = 1000000000;
};

} /* namespace msl */
