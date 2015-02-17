/*
 * $Id: MSLFootballField.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include <MSLFootballField.h>
#include <iostream>

namespace msl {

	double MSLFootballField::FieldLength = 11200.0;
	double MSLFootballField::FieldWidth = 8000.0;
	double MSLFootballField::GoalAreaWidth = 1200.0;
	double MSLFootballField::GoalAreaLength = 4000.0;
	double MSLFootballField::GoalInnerAreaLength = 3000.0;
	double MSLFootballField::GoalInnerAreaWidth = 700.0;
	double MSLFootballField::CornerCircleRadius = 350.0;
	double MSLFootballField::MiddleCircleRadius = 1000.0;
	double MSLFootballField::LineWidth = 75.0;
	double MSLFootballField::GoalWidth = 2000.0;
	bool MSLFootballField::GoalInnerAreaExists = false;
	bool MSLFootballField::CornerCircleExists = false;

	MSLFootballField *  MSLFootballField::instance = NULL;

	MSLFootballField::MSLFootballField() : sc() {

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

		std::cout << "MSLFootballField::FieldLength = " << FieldLength << std::endl;
		std::cout << "MSLFootballField::FieldWidth = " << FieldWidth << std::endl;
		std::cout << "MSLFootballField::GoalAreaWidth = " << GoalAreaWidth << std::endl;
		std::cout << "MSLFootballField::GoalAreaLength = " << GoalAreaLength << std::endl;
		std::cout << "MSLFootballField::MiddleCircleRadius = " << MiddleCircleRadius << std::endl;
		std::cout << "MSLFootballField::GoalInnerAreaWidth = " << GoalInnerAreaWidth << std::endl;
		std::cout << "MSLFootballField::GoalInnerAreaLength = " << GoalInnerAreaLength << std::endl;
		std::cout << "MSLFootballField::CornerCircleRadius = " << CornerCircleRadius << std::endl;
		std::cout << "MSLFootballField::LineWidth = " << LineWidth << std::endl;
		std::cout << "MSLFootballField::GoalInnerAreaExists = " << GoalInnerAreaExists << std::endl;
		std::cout << "MSLFootballField::CornerCircleExists = " << CornerCircleExists << std::endl;

	}


	MSLFootballField::~MSLFootballField(){
	
	
	}
	

	MSLFootballField * MSLFootballField::getInstance(){

		if(instance == NULL){
			instance = new MSLFootballField();
		}

		return instance;

	}
}


