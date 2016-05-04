/*
 * MovementQuery.cpp
 *
 *  Created on: Apr 27, 2016
 *      Author: Carpe Noctem
 */

#include "robotmovement/MovementQuery.h"

MovementQuery::MovementQuery()
{
	egoAlignPoint = nullptr;
	egoDestinationPoint = nullptr;
	additionalPoints = nullptr;
	fast = false;
	dribble = false;
	curRotDribble = 0;
	lastRotDribbleErr = 0;
	snapDistance = 0;
//	translation = 0;
	angleTolerance = 0;
	teamMatePosition = nullptr;

}

MovementQuery::~MovementQuery()
{

}

