/*
 * $Id: LocalizeDebug.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "LocalizeDebug.h"

#include "FootballField.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

LocalizeDebug::LocalizeDebug()
{

    FootballField::getInstance();

    init();
}

LocalizeDebug::~LocalizeDebug()
{

    cleanup();
}

void LocalizeDebug::init()
{

    Lines[0][0][0] = FootballField::FieldLength / 2.0;
    Lines[0][0][1] = FootballField::FieldWidth / 2.0;
    Lines[0][1][0] = FootballField::FieldLength / 2.0;
    Lines[0][1][1] = -FootballField::FieldWidth / 2.0;

    Lines[1][0][0] = -FootballField::FieldLength / 2.0;
    Lines[1][0][1] = FootballField::FieldWidth / 2.0;
    Lines[1][1][0] = -FootballField::FieldLength / 2.0;
    Lines[1][1][1] = -FootballField::FieldWidth / 2.0;

    Lines[2][0][0] = FootballField::FieldLength / 2.0;
    Lines[2][0][1] = FootballField::FieldWidth / 2.0;
    Lines[2][1][0] = -FootballField::FieldLength / 2.0;
    Lines[2][1][1] = FootballField::FieldWidth / 2.0;

    Lines[3][0][0] = FootballField::FieldLength / 2.0;
    Lines[3][0][1] = -FootballField::FieldWidth / 2.0;
    Lines[3][1][0] = -FootballField::FieldLength / 2.0;
    Lines[3][1][1] = -FootballField::FieldWidth / 2.0;

    Lines[4][0][0] = FootballField::FieldLength / 2.0 - FootballField::GoalAreaWidth;
    Lines[4][0][1] = FootballField::GoalAreaLength / 2.0;
    Lines[4][1][0] = FootballField::FieldLength / 2.0 - FootballField::GoalAreaWidth;
    Lines[4][1][1] = -FootballField::GoalAreaLength / 2.0;

    Lines[5][0][0] = FootballField::FieldLength / 2.0;
    Lines[5][0][1] = FootballField::GoalAreaLength / 2.0;
    Lines[5][1][0] = FootballField::FieldLength / 2.0 - FootballField::GoalAreaWidth;
    Lines[5][1][1] = FootballField::GoalAreaLength / 2.0;

    Lines[6][0][0] = FootballField::FieldLength / 2.0;
    Lines[6][0][1] = -FootballField::GoalAreaLength / 2.0;
    Lines[6][1][0] = FootballField::FieldLength / 2.0 - FootballField::GoalAreaWidth;
    Lines[6][1][1] = -FootballField::GoalAreaLength / 2.0;

    Lines[7][0][0] = -FootballField::FieldLength / 2.0 + FootballField::GoalAreaWidth;
    Lines[7][0][1] = FootballField::GoalAreaLength / 2.0;
    Lines[7][1][0] = -FootballField::FieldLength / 2.0 + FootballField::GoalAreaWidth;
    Lines[7][1][1] = -FootballField::GoalAreaLength / 2.0;

    Lines[8][0][0] = -FootballField::FieldLength / 2.0;
    Lines[8][0][1] = FootballField::GoalAreaLength / 2.0;
    Lines[8][1][0] = -FootballField::FieldLength / 2.0 + FootballField::GoalAreaWidth;
    Lines[8][1][1] = FootballField::GoalAreaLength / 2.0;

    Lines[9][0][0] = -FootballField::FieldLength / 2.0;
    Lines[9][0][1] = -FootballField::GoalAreaLength / 2.0;
    Lines[9][1][0] = -FootballField::FieldLength / 2.0 + FootballField::GoalAreaWidth;
    Lines[9][1][1] = -FootballField::GoalAreaLength / 2.0;

    Lines[10][0][0] = 0.0;
    Lines[10][0][1] = -FootballField::FieldWidth / 2.0;
    Lines[10][1][0] = 0.0;
    Lines[10][1][1] = FootballField::FieldWidth / 2.0;

    Lines[11][0][0] = FootballField::FieldLength / 2.0 - FootballField::GoalInnerAreaWidth;
    Lines[11][0][1] = FootballField::GoalInnerAreaLength / 2.0;
    Lines[11][1][0] = FootballField::FieldLength / 2.0 - FootballField::GoalInnerAreaWidth;
    Lines[11][1][1] = -FootballField::GoalInnerAreaLength / 2.0;

    Lines[12][0][0] = FootballField::FieldLength / 2.0;
    Lines[12][0][1] = FootballField::GoalInnerAreaLength / 2.0;
    Lines[12][1][0] = FootballField::FieldLength / 2.0 - FootballField::GoalInnerAreaWidth;
    Lines[12][1][1] = FootballField::GoalInnerAreaLength / 2.0;

    Lines[13][0][0] = FootballField::FieldLength / 2.0;
    Lines[13][0][1] = -FootballField::GoalInnerAreaLength / 2.0;
    Lines[13][1][0] = FootballField::FieldLength / 2.0 - FootballField::GoalInnerAreaWidth;
    Lines[13][1][1] = -FootballField::GoalInnerAreaLength / 2.0;

    Lines[14][0][0] = -FootballField::FieldLength / 2.0 + FootballField::GoalInnerAreaWidth;
    Lines[14][0][1] = FootballField::GoalInnerAreaLength / 2.0;
    Lines[14][1][0] = -FootballField::FieldLength / 2.0 + FootballField::GoalInnerAreaWidth;
    Lines[14][1][1] = -FootballField::GoalInnerAreaLength / 2.0;

    Lines[15][0][0] = -FootballField::FieldLength / 2.0;
    Lines[15][0][1] = FootballField::GoalInnerAreaLength / 2.0;
    Lines[15][1][0] = -FootballField::FieldLength / 2.0 + FootballField::GoalInnerAreaWidth;
    Lines[15][1][1] = FootballField::GoalInnerAreaLength / 2.0;

    Lines[16][0][0] = -FootballField::FieldLength / 2.0;
    Lines[16][0][1] = -FootballField::GoalInnerAreaLength / 2.0;
    Lines[16][1][0] = -FootballField::FieldLength / 2.0 + FootballField::GoalInnerAreaWidth;
    Lines[16][1][1] = -FootballField::GoalInnerAreaLength / 2.0;
}

vector<pair<double, double>> LocalizeDebug::drawFieldForParticle(Particle particle, int number)
{

    int numberLPoints = 50;
    std::vector<std::pair<double, double>> field;

    int NUMBER_USED_LINES = (FootballField::GoalInnerAreaExists ? 17 : 11);

    for (int i = 0; i < NUMBER_USED_LINES; i++)
    {

        double rv[2];
        rv[0] = Lines[i][1][0] - Lines[i][0][0];
        rv[1] = Lines[i][1][1] - Lines[i][0][1];

        for (int j = 0; j < numberLPoints; j++)
        {

            double lambda = j * 1.0 / (numberLPoints - 1.0);
            LinePoint realPoint;
            realPoint.x = Lines[i][0][0] + lambda * rv[0];
            realPoint.y = Lines[i][0][1] + lambda * rv[1];

            LinePoint mappedPoint;
            double x = realPoint.x - particle.posx;
            double y = realPoint.y - particle.posy;
            double angle = atan2(y, x) - particle.heading;
            double dist = sqrt(x * x + y * y);

            mappedPoint.x = cos(angle) * dist;
            mappedPoint.y = sin(angle) * dist;

            field.push_back(std::make_pair(mappedPoint.x, mappedPoint.y));
        }
    }

    for (int i = 0; i < numberLPoints; i++)
    {

        double angleReal = i * 1.0 / (numberLPoints - 1.0) * 2.0 * M_PI;
        LinePoint realPoint;
        realPoint.x = FootballField::MiddleCircleRadius * cos(angleReal);
        realPoint.y = FootballField::MiddleCircleRadius * sin(angleReal);

        LinePoint mappedPoint;
        double x = realPoint.x - particle.posx;
        double y = realPoint.y - particle.posy;
        double angle = atan2(y, x) - particle.heading;
        double dist = sqrt(x * x + y * y);

        mappedPoint.x = cos(angle) * dist;
        mappedPoint.y = sin(angle) * dist;

        field.push_back(std::make_pair(mappedPoint.x, mappedPoint.y));
    }
    return field;
}

void LocalizeDebug::cleanup()
{
}
