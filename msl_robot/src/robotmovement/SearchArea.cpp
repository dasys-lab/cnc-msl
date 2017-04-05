/*
 * SearchArea.cpp
 *
 *  Created on: Feb 4, 2016
 *      Author: Stefan Jakob
 */

#include "msl_robot/robotmovement/SearchArea.h"

namespace msl
{

int SearchArea::counter = 0;
int SearchArea::maxNum = 500;

SearchArea::SearchArea()
{
    this->midP = geometry::CNPointAllo();
    this->langle = 0;
    this->hangle = 0;
    this->minDist = 0;
    this->maxDist = 0;
    this->center = geometry::CNPointAllo();
    this->ownPos = geometry::CNPositionAllo();
    this->val = 0;
}

SearchArea::~SearchArea()
{
}

bool SearchArea::compareTo(shared_ptr<SearchArea> a, shared_ptr<SearchArea> b)
{
    if (a == b)
    {
        return true;
    }
    if (a->val < b->val)
    {
        return false;
    }
    if (a->val > b->val)
    {
        return false;
    }
    if (a->langle < b->langle)
    {
        return false;
    }
    if (a->langle > b->langle)
    {
        return false;
    }
    if (a->minDist < b->minDist)
    {
        return false;
    }
    if (a->minDist > b->minDist)
    {
        return false;
    }
    return true;
}

SearchArea::SearchArea(double langle, double hangle, double minDist, double maxDist, geometry::CNPointAllo center,
                       geometry::CNPositionAllo ownPos)
{
    this->langle = langle;
    this->hangle = hangle;
    this->minDist = minDist;
    this->maxDist = maxDist;
    this->center = center;
    this->ownPos = ownPos;
    this->midP = geometry::CNPointAllo(center.x, center.y);
    this->midP.x += cos((hangle + langle) / 2) * (maxDist + minDist) / 2;
    this->midP.y += sin((hangle + langle) / 2) * (maxDist + minDist) / 2;
    this->val = 0;
}

//	int SearchArea::compareTo(shared_ptr<SearchArea> a)
//	{
////		if (a == this)
////		{
////			return 0;
////		}
//		if (a->val < this->val)
//		{
//			return -1;
//		}
//		if (a->val > this->val)
//		{
//			return 1;
//		}
//		if (a->langle < this->langle)
//		{
//			return -1;
//		}
//		if (a->langle > this->langle)
//		{
//			return 1;
//		}
//		if (a->minDist < this->minDist)
//		{
//			return -1;
//		}
//		if (a->minDist > this->minDist)
//		{
//			return 1;
//		}
//		return 0;
//
//	}
}
/* namespace msl */
