/*
 * $Id: FootballField.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "FootballField.h"
#include <iostream>

double FootballField::FieldLength = 11200.0;
double FootballField::FieldWidth = 8000.0;
double FootballField::GoalAreaWidth = 1200.0;
double FootballField::GoalAreaLength = 4000.0;
double FootballField::GoalInnerAreaLength = 3000.0;
double FootballField::GoalInnerAreaWidth = 700.0;
double FootballField::CornerCircleRadius = 350.0;
double FootballField::MiddleCircleRadius = 1000.0;
double FootballField::LineWidth = 75.0;
double FootballField::GoalWidth = 2000.0;
bool FootballField::GoalInnerAreaExists = false;
bool FootballField::CornerCircleExists = false;
string FootballField::CurrentField = "";

FootballField *  FootballField::instance = NULL;

FootballField::FootballField() : sc() {

	this->sc = SystemConfig::getInstance();
	this->CurrentField = (*this->sc)["FootballField"]->get<string>("FootballField", "CurrentField", NULL);
	FieldLength = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "FieldLength", NULL);
	FieldWidth = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "FieldWidth", NULL);
	GoalAreaWidth = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "PenaltyAreaXSize", NULL);
	GoalAreaLength = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "PenaltyAreaYSize", NULL);
	MiddleCircleRadius = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "MiddleCircleRadius", NULL);
	GoalInnerAreaWidth = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "GoalAreaXSize", NULL);
	GoalInnerAreaLength = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "GoalAreaYSize", NULL);
	CornerCircleRadius = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "CornerCircleRadius", NULL);
	LineWidth = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "LineWidth", NULL);
	GoalWidth = (*this->sc)["FootballField"]->get<double>("FootballField", CurrentField.c_str(), "GoalWidth", NULL);
	GoalInnerAreaExists = (*this->sc)["FootballField"]->get<bool>("FootballField", CurrentField.c_str(), "GoalInnerAreaExists", NULL);
	CornerCircleExists = (*this->sc)["FootballField"]->get<bool>("FootballField", CurrentField.c_str(), "CornerCircleExists", NULL);

	std::cout << "MSLFootballField::CurrentField = " << CurrentField << std::endl;
	std::cout << "FootballField::FieldLength = " << FieldLength << std::endl;
	std::cout << "FootballField::FieldWidth = " << FieldWidth << std::endl;
	std::cout << "FootballField::GoalAreaWidth = " << GoalAreaWidth << std::endl;
	std::cout << "FootballField::GoalAreaLength = " << GoalAreaLength << std::endl;
	std::cout << "FootballField::MiddleCircleRadius = " << MiddleCircleRadius << std::endl;
	std::cout << "FootballField::GoalInnerAreaWidth = " << GoalInnerAreaWidth << std::endl;
	std::cout << "FootballField::GoalInnerAreaLength = " << GoalInnerAreaLength << std::endl;
	std::cout << "FootballField::CornerCircleRadius = " << CornerCircleRadius << std::endl;
	std::cout << "FootballField::LineWidth = " << LineWidth << std::endl;
	std::cout << "FootballField::GoalInnerAreaExists = " << GoalInnerAreaExists << std::endl;
	std::cout << "FootballField::CornerCircleExists = " << CornerCircleExists << std::endl;
	
}


FootballField::~FootballField(){


}


FootballField * FootballField::getInstance(){
	
	if(instance == NULL){
		instance = new FootballField();
	}
	
	return instance;

}



