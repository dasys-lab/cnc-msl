/*
 * $Id: ConfigHelper.cpp 1935 2007-03-19 19:50:12Z phbaer $
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
#include "ConfigHelper.h"
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>


ConfigElement::ConfigElement(std::string name_){

	name = name_;

}


ConfigElement::~ConfigElement(){


}



void ConfigElement::printContents(){

	std::map<std::string, std::string>::const_iterator first = Values.begin();
	std::map<std::string, std::string>::const_iterator last = Values.end();

	for(;first!=last;++first){

		std::cout << first->first << " = " << first->second << std::endl;

	}

	std::map<std::string, ConfigElement*>::const_iterator cfirst = childrenMap.begin();
	std::map<std::string, ConfigElement*>::const_iterator clast = childrenMap.end();

	for(;cfirst!=clast;++cfirst){

		std::cout << "ChildElement: " << cfirst->second->name << std::endl;
		cfirst->second->printContents();

	}

}



ConfigHelper::ConfigHelper(std::string configName){

	char buffer[256];
	gethostname(buffer, 256);

	std::string hostname(buffer);
	std::string es_root(getenv("ES_ROOT"));
	
	std::string filename = es_root + "/etc/" + hostname + "/" + configName;

	printf("FileName %s\n", filename.c_str());

	std::ifstream in;

	in.open(filename.c_str());
	if(!in.good()){
		printf("Host specific config %s not found !!!\n", filename.c_str());
		in.close();
		filename = es_root + "/etc/" + configName;
		printf("Using file %s!!!\n", filename.c_str());
		in.open(filename.c_str());
		
	}


	init(in);

}


ConfigHelper::~ConfigHelper(){

	cleanup();

}


bool ConfigHelper::parseSocketSpec(std::string & socketSpec, std::string & socketType, std::string & addr, int & port){

	unsigned int posD = socketSpec.find(':');
	socketType = socketSpec.substr(0, posD);
	
	if(socketType == "udp"){

		unsigned int posA = socketSpec.find('@');
		port = atoi(socketSpec.substr(posD+1,posA).c_str());

		addr = socketSpec.substr(posA+1);
		

	}
	else if(socketType == "unix"){

		addr = socketSpec.substr(posD+1);
		port = -1;

	}
	else
		return false;


	return true;


}

std::string ConfigHelper::getFileName(std::string & path){

	std::string filename;

	bool searchFinished = false;
	unsigned int startPos = 0;

	while(!searchFinished){

		unsigned long posD = path.find('$', startPos);
		if(posD == std::string::npos){
			filename = filename + path.substr(startPos, path.size() - startPos);
			searchFinished = true;
		}
		else {

			filename = filename + path.substr(startPos, posD - startPos);
			unsigned int posB = path.find(')', startPos);
			std::string env = path.substr(posD + 2, posB - posD - 2);
			startPos = posB + 1;
			filename = filename + std::string(getenv(env.c_str()));

		}



	}

	return filename;


}


void ConfigHelper::init(std::ifstream & in){

	rootElement = new ConfigElement("ConfigRoot");

	std::string line;
	std::string realLine;
	while(std::getline(in, line)){

		if(line.size() <= 0)
			continue;

		realLine = trim(line);

		if(realLine[0] == '#'){
			continue;
		}

		if(realLine[0] == '[' && realLine[1] != '!' && realLine[realLine.size()-1] == ']'){

			handleSection(realLine, in, rootElement);

		}


		else if(realLine[0] == '[' && realLine[1] == '!' && realLine[realLine.size()-1] == ']'){

			break;

		}
		else{
			unsigned int posEqual = realLine.find('=');

			if(posEqual <= 0 || posEqual >= realLine.size())
				continue;

			std::string name  = trim(realLine.substr(0,posEqual));
			std::string value = trim(realLine.substr(posEqual+1));

			rootElement->Values[name] = value;

		}

	}

}

void ConfigHelper::handleSection(std::string sectionLine, std::ifstream & in, ConfigElement * parent){

	std::string sectionName = trim(sectionLine.substr(1, sectionLine.size()-2));
	ConfigElement * childElement = new ConfigElement(sectionName);
	parent->childrenMap[sectionName] = childElement;
	parent->children.push_back(childElement);
	
	std::string line;
	std::string realLine;
	while(std::getline(in, line)){

		if(line.size() <= 0)
			continue;

		realLine = trim(line);

		if(realLine[0] == '#'){

			continue;
		}

		if(realLine[0] == '[' && realLine[1] != '!' && realLine[realLine.size()-1] == ']'){


			handleSection(realLine, in, childElement);

		}


		else if(realLine[0] == '[' && realLine[1] == '!' && realLine[realLine.size()-1] == ']'){
			std::string argName = trim(realLine.substr(2, realLine.size()-3));
			if(argName != sectionName){
				std::cout << "ConfigHelper: Nesting Error !!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
				std::cout << "SectionName: " << sectionName << std::endl;
			}
			break;

		}
		else{
			unsigned int posEqual = realLine.find('=');

			if(posEqual <= 0 || posEqual >= realLine.size())
				continue;

			std::string name  = trim(realLine.substr(0,posEqual));
			std::string value = trim(realLine.substr(posEqual+1));

			childElement->Values[name] = value;

		}

	}


}


void ConfigHelper::cleanup(){

	if(rootElement != NULL)
		delete rootElement;

}


std::string ConfigHelper::trim(std::string str){

	char * delims = " \t\r\n";
	std::string result(str);
	std::string::size_type index = result.find_last_not_of(delims);
	if(index != std::string::npos)
	result.erase(++index);

	index = result.find_first_not_of(delims);
	if(index != std::string::npos)
		result.erase(0, index);
	else
		result.erase();

	return result;


}

void ConfigHelper::printConfig(){

	rootElement->printContents();


}
