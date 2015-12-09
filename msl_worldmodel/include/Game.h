/*
 * Game.h
 *
 *  Created on: Feb 24, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_GAME_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_GAME_H_


#include <ros/ros.h>
#include <msl_msgs/RefBoxCommand.h>
#include <MSLEnums.h>
#include <robot_control/RobotCommand.h>
#include <mutex>
#include "GameState.h"
#include "RingBuffer.h"
#include "InformationElement.h"
#include "SystemConfig.h"
#include "Rules.h"

using namespace supplementary;

using namespace std;

namespace msl
{

	class MSLWorldModel;
	class Game
	{
	public:
		Game(MSLWorldModel* wm, int ringBufferLength);
		virtual ~Game();
		void onRobotCommand(robot_control::RobotCommandPtr msg);
		void onRefBoxCommand(msl_msgs::RefBoxCommandPtr msg);
		shared_ptr<msl_msgs::RefBoxCommand> getRefBoxCommand(int index);
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

		Rules rules;

	private:
		MSLWorldModel* wm;
		Situation situation;
		ros::NodeHandle n;
		ros::AsyncSpinner* spinner;
		ros::Subscriber refBoxCommandSub;
		ros::Subscriber robotCommandSub;
		RingBuffer<InformationElement<msl_msgs::RefBoxCommand>> refBoxCommand;
		mutex refereeMutex;
		mutex situationChecker;
		mutex goalMutex;
		SystemConfig* sc;
		int ownGoal;
		int oppGoal;
		unsigned long timeSinceStart;
		GameState gameState;
		int teamMateWithBall;
		bool mayScore;
		void setMayScore();
	};

} /* namespace alica */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_GAME_H_ */
