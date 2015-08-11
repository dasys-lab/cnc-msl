/*
 * XMLProtocolParser.cpp
 *
 *  Created on: May 28, 2015
 *      Author: Stephan Opfer
 */

#include "rqt_msl_refbox/XMLProtocolParser.h"
#include <iostream>

namespace rqt_msl_refbox
{

	XMLProtocolParser::XMLProtocolParser(GameData* gameData)
	{
		this->gameData = gameData;
		this->cyan =  false;
		this->magenta =  false;
	}

	XMLProtocolParser::~XMLProtocolParser()
	{
		// TODO Auto-generated destructor stub
	}

	void XMLProtocolParser::handle(tinyxml2::XMLElement* element)
	{
		processElement(element);
		tinyxml2::XMLElement* curChild = nullptr;
		tinyxml2::XMLElement* nextSibling = nullptr;

		curChild = element->FirstChildElement();
		nextSibling = element->NextSiblingElement();
		if(curChild != nullptr)
			handle(curChild);
		if(nextSibling != nullptr)
			handle(nextSibling);

	}

	void XMLProtocolParser::processElement(tinyxml2::XMLElement* curChild)
	{
		const char* val = curChild->Value();
		std::string str(val);

		if(str.compare("Referee") == 0)
		{
			handleReferee(curChild);
		}
		else if (str.compare("StageChange") == 0)
		{
			handleStageChange(curChild);
		}
		else if(str.compare("GameStart") == 0)
		{
			handleGameStart(curChild);
		}
		else if(str.compare("GameStop") == 0)
		{
			handleGameStop(curChild);
		}
		else if (str.compare("KickOff") == 0)
		{
			handleKickOff(curChild);
		}
		else if(str.compare("DroppedBall") == 0)
		{
			handleDroppedBall(curChild);
		}
		else if (str.compare("FreeKick") == 0)
		{
			handleFreeKick(curChild);
		}
		else if (str.compare("GoalKick") == 0)
		{
			handleGoalKick(curChild);
		}
		else if (str.compare("ThrowIn") == 0)
		{
			handleThrowIn(curChild);
		}
		else if (str.compare("Corner") == 0)
		{
			handleCornerKick(curChild);
		}
		else if (str.compare("Penalty") == 0)
		{
			handlePenalty(curChild);
		}
		else if(str.compare("Parking") == 0)
		{
			handleParking(curChild);
		}
		else if(str.compare("RefboxEvent") == 0)
		{
			//DO NOTHING THIS CUZ OF --> <RefboxEvent xmlns="http://www.robocup.org/MSL/refbox/protocol2010">
		}
		else if(str.compare("GameInfo") == 0)
		{
			handleGameInfo(curChild);
		}
		else if (str.compare("TeamData") == 0)
		{
			handleTeamData(curChild);
		}
		else if(str.compare("Setup") == 0)
		{
			handleSetup(curChild);
		}
		else if(str.compare("Player") == 0)
		{
			handlePlayer(curChild);
		}
		else if(str.compare("PlayerOut") == 0)
		{
			handlePlayerOut(curChild);
		}
		else if(str.compare("PlayerIn") == 0)
		{
			handlePlayerIn(curChild);
		}
		else if(str.compare("GoalAwarded") == 0)
		{
			handleGoalAwarded(curChild);
		}
		else if(str.compare("CardAwarded") == 0)
		{
			handleCardAwarded(curChild);
		}
		else
		{
			std::cerr << "RQTREFBOX: CANNOT PARSE-> " << val << std::endl;
		}
	}

	/*##### Handle methods for all xml Elements #####*/

	void XMLProtocolParser::handleReferee(tinyxml2::XMLElement* curChild)
	{
		const tinyxml2::XMLAttribute* attr = curChild->FirstAttribute();
		while(attr != nullptr)
		{
			const char* valAttr = attr->Value();
			const char* valName = attr->Name();
			std::string strAttr(valAttr);
			std::string strName(valName);

			if(strName.compare("time") == 0)
			{
				this->gameData->refBox->lbl_time->setText(QString(valAttr));
			}
			else if(strName.compare("stage") == 0)
			{
				this->gameData->refBox->lbl_stage->setText(QString(valAttr));
			}
			attr = attr->Next();
		}
	}
	void XMLProtocolParser::handleStageChange(tinyxml2::XMLElement* curChild)
	{
		//TODO SUPER KACKE SOWAS
	}
	void XMLProtocolParser::handleGameStop(tinyxml2::XMLElement* curChild)
	{
		this->gameData->sendStop();
		this->gameData->refBox->RefLog->append("Game Stop");
	}

	void XMLProtocolParser::handleGameStart(tinyxml2::XMLElement* curChild)
	{
		this->gameData->sendStart();
		this->gameData->refBox->RefLog->append("Game Start");
	}

	void XMLProtocolParser::handleKickOff(tinyxml2::XMLElement* curChild)
	{
		const tinyxml2::XMLAttribute* attr = curChild->FirstAttribute();
		const char* val = attr->Value();
		std::string str(val);

		if(str.compare("Cyan") == 0)
		{
			this->gameData->sendCyanKickOff();
			this->gameData->refBox->RefLog->append("KickOff Cyan");
		}
		else if (str.compare("Magenta") == 0)
		{
			this->gameData->sendMagentaKickOff();
			this->gameData->refBox->RefLog->append("KickOff Magenta");
		}
	}

	void XMLProtocolParser::handleDroppedBall(tinyxml2::XMLElement* curChild)
	{
		this->gameData->sendDroppedBall();
		this->gameData->refBox->RefLog->append("Dropped Ball");
	}

	void XMLProtocolParser::handleFreeKick(tinyxml2::XMLElement* curChild)
	{
		const tinyxml2::XMLAttribute* attr = curChild->FirstAttribute();
		const char* val = attr->Value();
		std::string str(val);

		if(str.compare("Cyan") == 0)
		{
			this->gameData->sendCyanFreeKick();
			this->gameData->refBox->RefLog->append("FreeKick Cyan");
		}
		else if (str.compare("Magenta") == 0)
		{
			this->gameData->sendMagentaFreeKick();
			this->gameData->refBox->RefLog->append("FreeKick Magenta");
		}

	}

	void XMLProtocolParser::handleGoalKick(tinyxml2::XMLElement* curChild)
	{
		const tinyxml2::XMLAttribute* attr = curChild->FirstAttribute();
		const char* val = attr->Value();
		std::string str(val);

		if(str.compare("Cyan") == 0)
		{
			this->gameData->sendCyanGoalKick();
			this->gameData->refBox->RefLog->append("GoalKick Cyan");
		}
		else if (str.compare("Magenta") == 0)
		{
			this->gameData->sendMagentaGoalKick();
			this->gameData->refBox->RefLog->append("GoalKick Magenta");
		}
	}

	void XMLProtocolParser::handleThrowIn(tinyxml2::XMLElement* curChild)
	{
		const tinyxml2::XMLAttribute* attr = curChild->FirstAttribute();
		const char* val = attr->Value();
		std::string str(val);

		if(str.compare("Cyan") == 0)
		{
			this->gameData->sendCyanThrownin();
			this->gameData->refBox->RefLog->append("ThrowIn Cyan");
		}
		else if (str.compare("Magenta") == 0)
		{
			this->gameData->sendMagentaThrownin();
			this->gameData->refBox->RefLog->append("ThrowIn Magenta");
		}

	}

	void XMLProtocolParser::handleCornerKick(tinyxml2::XMLElement* curChild)
	{
		const tinyxml2::XMLAttribute* attr = curChild->FirstAttribute();
		const char* val = attr->Value();
		std::string str(val);

		if(str.compare("Cyan") == 0)
		{
			this->gameData->sendCyanCornerKick();
			this->gameData->refBox->RefLog->append("CornerKick Cyan");
		}
		else if (str.compare("Magenta") == 0)
		{
			this->gameData->sendMagentaCornerKick();
			this->gameData->refBox->RefLog->append("CornerKick Magenta");
		}

	}

	void XMLProtocolParser::handlePenalty(tinyxml2::XMLElement* curChild)
	{
		const tinyxml2::XMLAttribute* attr = curChild->FirstAttribute();
		const char* val = attr->Value();
		std::string str(val);

		if(str.compare("Cyan") == 0)
		{
			this->gameData->sendCyanPenalty();
			this->gameData->refBox->RefLog->append("Penalty Cyan");
		}
		else if (str.compare("Magenta") == 0)
		{
			this->gameData->sendMagentaPenalty();
			this->gameData->refBox->RefLog->append("Penalty Magenta");
		}
	}

	void XMLProtocolParser::handleParking(tinyxml2::XMLElement* curChild)
	{
		this->gameData->sendParking();
		this->gameData->refBox->RefLog->append("Parking");
	}

	void XMLProtocolParser::handleGameInfo(tinyxml2::XMLElement* curChild)
	{
		const tinyxml2::XMLAttribute* attr = curChild->FirstAttribute();
		while(attr != nullptr)
		{
			const char* valAttr = attr->Value();
			const char* valName = attr->Name();
			std::string strAttr(valAttr);
			std::string strName(valName);

			if(strName.compare("time") == 0)
			{
				this->gameData->refBox->lbl_time->setText(QString(valAttr));
			}
			else if(strName.compare("stage") == 0)
			{
				this->gameData->refBox->lbl_stage->setText(QString(valAttr));
			}
			attr = attr->Next();

		}
	}

	void XMLProtocolParser::handleTeamData(tinyxml2::XMLElement* curChild)
	{
		const tinyxml2::XMLAttribute* attr = curChild->FirstAttribute();
		while(attr != nullptr)
		{
			const char* valAttr = attr->Value();
			const char* valName = attr->Name();
			std::string strAttr(valAttr);
			std::string strName(valName);

			if(strName.compare("team") == 0)
			{
				if(strAttr.compare("Cyan") == 0)
				{
					this->cyan = true;
					this->magenta =  false;
				}
				else if(strAttr.compare("Magenta") == 0)
				{
					this->cyan = false;
					this->magenta =  true;
				}
			}
			attr = attr->Next();
		}

	}

	void XMLProtocolParser::handleSetup(tinyxml2::XMLElement* curChild)
	{
		const tinyxml2::XMLAttribute* attr = curChild->FirstAttribute();
		while(attr != nullptr)
		{
			const char* valAttr = attr->Value();
			const char* valName = attr->Name();
			std::string strAttr(valAttr);
			std::string strName(valName);

			if(strName.compare("name") == 0)
			{
				//TODO NEUER TAB
			}
			else if(strName.compare("leader") == 0)
			{
				//TODO NEUER TAB
			}

			attr = attr->Next();
		}
	}

	void XMLProtocolParser::handlePlayer(tinyxml2::XMLElement* curChild)
	{
		const tinyxml2::XMLAttribute* attr = curChild->FirstAttribute();
		while(attr != nullptr)
		{
			const char* valAttr = attr->Value();
			const char* valName = attr->Name();
			std::string strAttr(valAttr);
			std::string strName(valName);

			if(strName.compare("name") == 0)
			{
				//TODO NEUER TAB
			}
			else if(strName.compare("inField") == 0)
			{
				//TODO NEUER TAB
			}

			attr = attr->Next();
		}
	}

	void XMLProtocolParser::handlePlayerOut(tinyxml2::XMLElement* curChild)
	{
		const tinyxml2::XMLAttribute* attr = curChild->FirstAttribute();
		while(attr != nullptr)
		{
			const char* valAttr = attr->Value();
			const char* valName = attr->Name();
			std::string strAttr(valAttr);
			std::string strName(valName);

			if(strName.compare("team") == 0)
			{
				//TODO NEUER TAB
			}
			else if(strName.compare("reason") == 0)
			{
				//TODO NEUER TAB
			}

			attr = attr->Next();
		}
	}

	void XMLProtocolParser::handlePlayerIn(tinyxml2::XMLElement* curChild)
	{
		const tinyxml2::XMLAttribute* attr = curChild->FirstAttribute();
		while(attr != nullptr)
		{
			const char* valAttr = attr->Value();
			const char* valName = attr->Name();
			std::string strAttr(valAttr);
			std::string strName(valName);

			if(strName.compare("team") == 0)
			{
				//TODO NEUER TAB
			}
			else if(strName.compare("reason") == 0)
			{
				//TODO NEUER TAB
			}

			attr = attr->Next();
		}
	}

	void XMLProtocolParser::handleGoalAwarded(tinyxml2::XMLElement* curChild)
	{
		const tinyxml2::XMLAttribute* attr = curChild->FirstAttribute();
		while(attr != nullptr)
		{
			const char* valAttr = attr->Value();
			const char* valName = attr->Name();
			std::string strAttr(valAttr);
			std::string strName(valName);

			if(strName.compare("team") == 0)
			{
				//TODO NEUER TAB
			}
			else if(strName.compare("player") == 0)
			{
				//TODO NEUER TAB
			}
			else if(strName.compare("own") == 0)
			{
				//TODO NEUER TAB
			}
			else if(strName.compare("stage") == 0)
			{
				//TODO NEUER TAB
			}
			else if(strName.compare("time") == 0)
			{
				//TODO NEUER TAB
			}
			attr = attr->Next();
		}

	}

	void XMLProtocolParser::handleCardAwarded(tinyxml2::XMLElement* curChild)
	{
		const tinyxml2::XMLAttribute* attr = curChild->FirstAttribute();
		while(attr != nullptr)
		{
			const char* valAttr = attr->Value();
			const char* valName = attr->Name();
			std::string strAttr(valAttr);
			std::string strName(valName);

			if(strName.compare("team") == 0)
			{
				//TODO NEUER TAB
			}
			else if(strName.compare("player") == 0)
			{
				//TODO NEUER TAB
			}
			else if(strName.compare("color") == 0)
			{
				//TODO NEUER TAB
			}
			else if(strName.compare("number") == 0)
			{
				//TODO NEUER TAB
			}
			else if(strName.compare("time") == 0)
			{
				//TODO NEUER TAB
			}
			else if(strName.compare("stage") == 0)
			{
				//TODO NEUER TAB
			}
			attr = attr->Next();
		}
	}

} /* namespace rqt_pm_control */
