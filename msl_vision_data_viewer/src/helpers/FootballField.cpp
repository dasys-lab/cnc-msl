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

FootballField *  FootballField::instance = NULL;

FootballField::FootballField() : sc() {

	this->sc = SystemConfig::getInstance();

	FieldLength = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "FieldLength", NULL);
	FieldWidth = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "FieldWidth", NULL);
	GoalAreaWidth = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "PenaltyAreaXSize", NULL);
	GoalAreaLength = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "PenaltyAreaYSize", NULL);
	MiddleCircleRadius = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "MiddleCircleRadius", NULL);
	GoalInnerAreaWidth = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "GoalAreaXSize", NULL);
	GoalInnerAreaLength = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "GoalAreaYSize", NULL);
	CornerCircleRadius = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "CornerCircleRadius", NULL);
	LineWidth = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "LineWidth", NULL);
	GoalWidth = (*this->sc)["Globals"]->get<double>("Globals", "FootballField", "GoalWidth", NULL);
	GoalInnerAreaExists = (*this->sc)["Globals"]->get<bool>("Globals", "FootballField", "GoalInnerAreaExists", NULL);
	CornerCircleExists = (*this->sc)["Globals"]->get<bool>("Globals", "FootballField", "CornerCircleExists", NULL);

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



