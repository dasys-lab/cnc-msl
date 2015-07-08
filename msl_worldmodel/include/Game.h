/*
 * Game.h
 *
 *  Created on: Feb 24, 2015
 *      Author: Stefan Jakob
 */

#ifndef CNC_MSL_MSL_WORLDMODEL_INCLUDE_GAME_H_
#define CNC_MSL_MSL_WORLDMODEL_INCLUDE_GAME_H_


#include <ros/ros.h>
#include <msl_msgs/RefereeBoxInfoBody.h>
#include <mutex>
#include "Situation.h"
#include "GameState.h"

#include "SystemConfig.h"

using namespace supplementary;

using namespace std;

namespace msl
{

	class MSLWorldModel;
	class Game
	{
	public:
		Game(MSLWorldModel* wm);
		virtual ~Game();
		void onRefereeBoxInfoBody(msl_msgs::RefereeBoxInfoBodyPtr msg);
		msl_msgs::RefereeBoxInfoBodyPtr getRefereeBoxInfoBody();
		bool checkSituation(Situation situation);
		Situation getCurrentSituation();
		Situation getLastSituation();
		int getOppGoal();
		int getOwnGoal();
		unsigned long getTimeSinceStart();
		Situation getSituation();
		GameState getGameState();
		void setGameState(GameState gameState);
		void updateGameState();
		bool isMayScore();

		string ownTeamColor;
		string ownGoalColor;
		long gameTime;

	private:
		MSLWorldModel* wm;
		Situation currentSituation;
		Situation lastSituation;
		ros::NodeHandle n;
		ros::AsyncSpinner* spinner;
		ros::Subscriber refereeBoxInfoBodySub;
		list<msl_msgs::RefereeBoxInfoBodyPtr> refereeBoxInfoBodyCommandData;
		mutex refereeMutex;
		mutex situationChecker;
		mutex goalMutex;
		SystemConfig* sc;
		int ownGoal;
		int oppGoal;
		unsigned long timeSinceStart;
		Situation lastActiveSituation;
		GameState gameState;
		int teamMateWithBall;
		void setMayScore();
		bool mayScore;
	};

} /* namespace alica */

#endif /* CNC_MSL_MSL_WORLDMODEL_INCLUDE_GAME_H_ */
