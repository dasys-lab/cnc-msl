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
}

MovementQuery::~MovementQuery()
{

}

