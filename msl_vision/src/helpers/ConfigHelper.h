/*
 * $Id: ConfigHelper.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef ConfigHelper_H
#define ConfigHelper_H


#include <string>
#include <fstream>
#include <vector>
#include <map>


class ConfigElement{

	public:
		ConfigElement(std::string name_);
		~ConfigElement();
		
		std::string name;
		std::map<std::string, std::string> Values;
		std::map<std::string, ConfigElement*> childrenMap;
		std::vector<ConfigElement*> children;

		void printContents();


};


class ConfigHelper{


	public:
		ConfigHelper(std::string configName);
		~ConfigHelper();

		static bool parseSocketSpec(std::string & socketSpec, std::string & socketType, std::string & addr, int  & port);
		static std::string getFileName(std::string & path);

		void printConfig();	

		ConfigElement * rootElement;

	protected:


		
		void init(std::ifstream & in);
		void cleanup();

		std::string trim(std::string str);
		void handleSection(std::string sectionLine, std::ifstream & in, ConfigElement * parent);



};



#endif

