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
	}

	XMLProtocolParser::~XMLProtocolParser()
	{
		// TODO Auto-generated destructor stub
	}

	void XMLProtocolParser::handle(tinyxml2::XMLElement* element)
	{
		tinyxml2::XMLElement* curChild = element;
		while (curChild != nullptr)
		{
			const char* val = curChild->Value();

			std::cout << "VAL: " << val << std::endl;

			curChild = curChild->FirstChildElement();
		}

	}

} /* namespace rqt_pm_control */
