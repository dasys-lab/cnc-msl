/*
 * EgoSearchArea.cpp
 *
 *  Created on: Feb 4, 2016
 *      Author: Stefan Jakob
 */

using namespace std;

#include "msl_robot/robotmovement/EgoSearchArea.h"
#include <MSLWorldModel.h>

namespace msl
{

EgoSearchArea::EgoSearchArea()
    : SearchArea()
{
}

EgoSearchArea::~EgoSearchArea()
{
}

void EgoSearchArea::init()
{
    daList = make_shared<vector<shared_ptr<EgoSearchArea>>>(maxNum);
    for (int i = 0; i < maxNum; i++)
    {
        daList->push_back(make_shared<EgoSearchArea>());
    }
    counter = 0;
}

shared_ptr<EgoSearchArea> EgoSearchArea::getAnArea(double langle, double hangle, double minDist, double maxDist, shared_ptr<geometry::CNPoint2D> center,
                                                   shared_ptr<geometry::CNPosition> ownPos)
{
    shared_ptr<EgoSearchArea> ret = daList->at(counter);
    counter = (counter + 1) % maxNum;
    ret->langle = langle;
    ret->hangle = hangle;
    ret->minDist = minDist;
    ret->maxDist = maxDist;
    ret->center = center;
    ret->ownPos = ownPos;
    ret->midP->x = center->x;
    ret->midP->y = center->y;
    ret->midP->x += cos((hangle + langle) / 2) * (maxDist + minDist) / 2;
    ret->midP->y += sin((hangle + langle) / 2) * (maxDist + minDist) / 2;
    // this.midP=WorldHelper.Allo2Ego(field.MapOutOfEnemyPenalty(WorldHelper.Ego2Allo(midP,ownPos)),ownPos);
    ret->alloP = ret->midP->egoToAllo(*ownPos);
    ret->val = 0;
    return ret;
}

EgoSearchArea::EgoSearchArea(double langle, double hangle, double minDist, double maxDist, shared_ptr<geometry::CNPoint2D> center,
                             shared_ptr<geometry::CNPosition> ownPos)
    : SearchArea(langle, hangle, minDist, maxDist, center, ownPos)
{
    this->alloP = this->midP->egoToAllo(*ownPos);
}

shared_ptr<vector<shared_ptr<SearchArea>>> EgoSearchArea::expand()
{
    shared_ptr<vector<shared_ptr<SearchArea>>> l = make_shared<vector<shared_ptr<SearchArea>>>(4);
    if (maxDist - minDist < 200)
        return l;
    // SearchArea tmp = new SearchArea(langle,(hangle+langle)/2,minDist,(minDist+maxDist)/2,center,ownPos);
    shared_ptr<EgoSearchArea> tmp = getAnArea(langle, (hangle + langle) / 2, minDist, (minDist + maxDist) / 2, center, ownPos);
    if (tmp->isValid())
    {
        l->push_back(tmp);
    }
    // tmp = new SearchArea((hangle+langle)/2,hangle,minDist,(minDist+maxDist)/2,center,ownPos);
    tmp = getAnArea((hangle + langle) / 2, hangle, minDist, (minDist + maxDist) / 2, center, ownPos);
    if (tmp->isValid())
    {
        l->push_back(tmp);
    }
    // tmp = new SearchArea(langle,(hangle+langle)/2,(minDist+maxDist)/2,maxDist,center,ownPos);
    tmp = getAnArea(langle, (hangle + langle) / 2, (minDist + maxDist) / 2, maxDist, center, ownPos);
    if (tmp->isValid())
    {
        l->push_back(tmp);
    }
    // tmp = new SearchArea((hangle+langle)/2,hangle,(minDist+maxDist)/2,maxDist,center,ownPos);
    tmp = getAnArea((hangle + langle) / 2, hangle, (minDist + maxDist) / 2, maxDist, center, ownPos);
    if (tmp->isValid())
    {
        l->push_back(tmp);
    }
    return l;
}
bool EgoSearchArea::isValid()
{
    return checkAlloPoint(this->alloP);
}

bool EgoSearchArea::checkAlloPoint(shared_ptr<geometry::CNPoint2D> ap)
{
    MSLWorldModel *wm = MSLWorldModel::get();
    if (wm->field->getFieldLength() / 2.0 - 1000 < ap->x)
    {
        return false;
    }
    if (-wm->field->getFieldLength() / 2.0 + 1000 > ap->x)
    {
        return false;
    }
    if (wm->field->getFieldWidth() / 2.0 < ap->y)
    {
        return false;
    }
    if (-wm->field->getFieldWidth() / 2.0 > ap->y)
    {
        return false;
    }
    if (wm->field->isInsideOppPenalty(ap, 100))
    {
        return false;
    }
    return true;
}
string EgoSearchArea::toString()
{
    stringstream ss;
    ss << "Angle: " << (langle * 180 / M_PI) << " " << (hangle * 180 / M_PI) << "\n"
       << "Dist: " << minDist << " " << maxDist << "\n"
       << "P : " << midP->x << " " << midP->y << "\n"
       << "Eval: " << val << "\n======" << endl;
    return ss.str();
}

string EgoSearchArea::toAlloString(shared_ptr<geometry::CNPosition> ownPos)
{
    shared_ptr<geometry::CNPoint2D> a = midP->egoToAllo(*ownPos);
    stringstream ss;
    ss << "Angle: " << (langle * 180 / M_PI) << " " << (hangle * 180 / M_PI) << "\n"
       << "Dist: " << minDist << " " << maxDist << "\n"
       << "P : " << a->x << " " << a->y << "\n"
       << "Eval: " << val << "\n======" << endl;
    return ss.str();
}
}
/* namespace msl */
