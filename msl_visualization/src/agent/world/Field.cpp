/*
 * Copyright (C) 2009-2015,
 * Intelligent Robotics and Intelligent Systems (IRIS) Lab
 * CAMBADA robotic soccer team – http://robotica.ua.pt/CAMBADA/
 * University of Aveiro, Portugal
 *
 * This file is part of the CAMBADA AGENT
 *
 * CAMBADA AGENT is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CAMBADA AGENT is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "Field.h"

using namespace cambada::geom;

namespace cambada{

/*Field::Field() {}*/

Field::Field( )
{
	this->length = 18;
	this->halfLength = this->length/2.;
	this->width = 12;
	this->halfWidth = this->width/2.;
	this->goalAreaWidth = 8;
	this->goalAreaLength = 2.5;
	this->goalAreaHalfWidth = this->goalAreaWidth/2.;
	this->goalAreaHalfLength = this->goalAreaLength/2.;
	this->penaltyAreaWidth = 6;
	this->penaltyAreaLength = 1;
	this->penaltyAreaHalfWidth = this->penaltyAreaWidth/2.;
	this->penaltyAreaHalfLength = this->penaltyAreaLength/2.;
	this->penaltyMarkerDistance = 3.5;
	this->ballDiameter = 0.25;
	this->ballRadius = this->ballDiameter/2.;
	this->borderLineThickness = 0.1;
	this->lineThickness = 0.1;
//	this->linesetType = config->getField("lineset_type");
	this->centerCircleRadius = 2;
	this->centerCircleDiameter = this->centerCircleRadius*2.;
	this->cornerArcRadius = 0.3;
	this->goalBandWidth = 2;
	this->goalHeight = 1;
	this->goalLength = 1;
	this->goalWidth = 2;
	this->goalHalfWidth = this->goalWidth/2;
//	this->zoneSlider = config->getField("zone_slider")/1000.;
	this->theNorth = 0;

	this->theirGoal = geom::Vec(0.0 , this->halfLength);
	this->ourGoal = geom::Vec(0.0 , -this->halfLength);

	//this->config = config;
}


bool Field::isInside(geom::Vec testPos, float outerMargin){
	if( (fabs(testPos.x) > (halfWidth + outerMargin)) || (fabs(testPos.y) > (halfLength + outerMargin) ) )
		return false;

	return true;
}

bool Field::isInsideTheirGoalArea(geom::Vec testPos, float outerMargin) {

	if ( (fabs(testPos.x) < (goalAreaHalfWidth + 0.25 + outerMargin) )
			&& ( testPos.y > ((halfLength - goalAreaLength) - 0.25 - outerMargin) ) )
		return true;

	return false;
}

bool Field::isInsideOurGoalArea(geom::Vec testPos, float outerMargin) {

	if ( (fabs(testPos.x) < (goalAreaHalfWidth + 0.25 + outerMargin) )
			&& ( testPos.y < ((-halfLength + goalAreaLength) + 0.25 + outerMargin) ) )
		return true;

	return false;
}

void Field::ourPenaltyAreaFilter( Vec& pos , float offset )
{
        const Vec areaP1 =
                        Vec( - (penaltyAreaHalfWidth)/2 - offset,
                                   - (penaltyAreaHalfLength)/2 + (penaltyAreaHalfLength) + offset);
        const Vec areaP2 =
                       Vec( + (penaltyAreaHalfWidth)/2 + offset,
                                   - (penaltyAreaHalfLength));

        XYRectangle area(areaP1,areaP2);

        if( area.is_inside(pos) )
                pos = area.adjust(pos);
}

bool Field::isNearOurPenaltyArea(Vec pointAbs, double distance)
{
	const Vec position = pointAbs;
	if ( (fabs(position.x) < (penaltyAreaWidth/2.0 + 0.25 + distance) )
	&& ( position.y < -((halfLength - penaltyAreaLength) - 0.25 - distance) ) )
		return true;

	return false;
}

void Field::theirGoalAreaFilter( geom::Vec& pos , float offset ) {
	const Vec areaP1 =
			Vec( -( penaltyAreaWidth/2.0) - offset ,
					penaltyAreaLength );

	const Vec areaP2 =
			Vec( + penaltyAreaWidth/2.0 + offset,
					penaltyAreaLength - penaltyAreaLength - offset);


	XYRectangle area(areaP1,areaP2);

	if( area.is_inside(pos) )
		pos = area.adjust(pos);
}

void Field::filter( geom::Vec& pos , float offset ) {
	const Vec areaP1 = Vec( -halfWidth -offset , -halfLength -offset );
	const Vec areaP2 = Vec( +halfWidth +offset , +halfLength +offset);

	XYRectangle area(areaP1,areaP2);

	if( area.is_inside(pos) )
		pos = area.adjust(pos);
}

} /* namespace cambada */
