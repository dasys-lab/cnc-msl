/*
 * XMLProtocolParser.h
 *
 *  Created on: May 28, 2015
 *      Author: Stephan Opfer
 */

#ifndef CNC_MSL_RQT_MSL_REFBOX_SRC_RQT_MSL_REFBOX_XMLPROTOCOLPARSER_H_
#define CNC_MSL_RQT_MSL_REFBOX_SRC_RQT_MSL_REFBOX_XMLPROTOCOLPARSER_H_

#include "rqt_msl_refbox/GameData.h"
#include "rqt_msl_refbox/tinyxml2.h"
#include <vector>
#include <string>

namespace rqt_msl_refbox
{
	class GameData;
	class XMLProtocolParser
	{
	public:
		XMLProtocolParser(GameData* gameData);
		virtual ~XMLProtocolParser();
		void handle(tinyxml2::XMLElement* element);
		void processElement(tinyxml2::XMLElement* curChild);


		//handleElement methods
		void handleReferee(tinyxml2::XMLElement* curChild);
		void handleStageChange(tinyxml2::XMLElement* curChild);
		void handleGameStop(tinyxml2::XMLElement* curChild);
		void handleGameStart(tinyxml2::XMLElement* curChild);
		void handleKickOff(tinyxml2::XMLElement* curChild);
		void handleDroppedBall(tinyxml2::XMLElement* curChild);
		void handleFreeKick(tinyxml2::XMLElement* curChild);
		void handleGoalKick(tinyxml2::XMLElement* curChild);
		void handleThrowIn(tinyxml2::XMLElement* curChild);
		void handleCornerKick(tinyxml2::XMLElement* curChild);
		void handlePenalty(tinyxml2::XMLElement* curChild);
		void handleParking(tinyxml2::XMLElement* curChild);
		void handleGameInfo(tinyxml2::XMLElement* curChild);
		void handleTeamData(tinyxml2::XMLElement* curChild);
		void handleSetup(tinyxml2::XMLElement* curChild);
		void handlePlayer(tinyxml2::XMLElement* curChild);
		void handlePlayerOut(tinyxml2::XMLElement* curChild);
		void handlePlayerIn(tinyxml2::XMLElement* curChild);
		void handleGoalAwarded(tinyxml2::XMLElement* curChild);
		void handleCardAwarded(tinyxml2::XMLElement* curChild);

	private:
		GameData* gameData;
		bool cyan;
		bool magenta;
		std::vector<std::string> cyanSetup;
		std::vector<std::string> magentaSetup;
		bool fillSetup(const char* valAttr);
		void fillTable(std::string name, std::string inField);
		void fillYellow(std::string team, std::string player, std::string color, std::string number);
		int goalCyan;
		int goalMagenta;
	};

} /* namespace rqt_pm_control */

#endif /* CNC_MSL_RQT_MSL_REFBOX_SRC_RQT_MSL_REFBOX_XMLPROTOCOLPARSER_H_ */
