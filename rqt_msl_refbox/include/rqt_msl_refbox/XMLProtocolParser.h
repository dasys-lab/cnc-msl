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

namespace rqt_msl_refbox
{
	class GameData;
	class XMLProtocolParser
	{
	public:
		XMLProtocolParser(GameData* gameData);
		virtual ~XMLProtocolParser();
		void handle(tinyxml2::XMLElement* element);


	private:
		GameData* gameData;
	};

} /* namespace rqt_pm_control */

#endif /* CNC_MSL_RQT_MSL_REFBOX_SRC_RQT_MSL_REFBOX_XMLPROTOCOLPARSER_H_ */
