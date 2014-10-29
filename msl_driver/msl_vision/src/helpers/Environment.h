/*
 * $Id: Environment.h 1531 2006-08-01 21:36:57Z phbaer $
 *
 *
 * Copyright 2005,2006 Carpe Noctem, Distributed Systems Group,
 * University of Kassel. All right reserved.
 *
 * The code is derived from the software contributed to Carpe Noctem by
 * the Carpe Noctem Team.
 *
 * The code is licensed under the Carpe Noctem Userfriendly BSD-Based
 * License (CNUBBL). Redistribution and use in source and binary forms,
 * with or without modification, are permitted provided that the
 * conditions of the CNUBBL are met.
 *
 * You should have received a copy of the CNUBBL along with this
 * software. The license is also available on our website:
 * http://carpenoctem.das-lab.net/license.txt
 *
 *
 * <description>
 */
#ifndef Environment_H
#define Environment_H

#include <string>
#include <SystemConfig.h>

using namespace castor;

class Environment{

	public:


		static Environment * getInstance();
		//std::string logFileName;

		void setLogFileName(std::string filename);
		void setLoggingMode(bool mode);

		std::string getLogFileName();
		bool getLoggingMode(); 


	private:

		SystemConfigPtr sc;

		std::string logFileName;
		bool loggingMode;

		static Environment * instance;
		Environment();
		~Environment();
		


};



#endif

