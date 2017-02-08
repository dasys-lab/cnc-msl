/*
 * KickCurve.cpp
 *
 *  Created on: Jul 13, 2015
 *      Author: Stefan Jakob
 */

#include <SystemConfig.h>
#include <msl_robot/kicker/KickCurve.h>

namespace msl
{

KickCurve::KickCurve(string section)
{
    supplementary::SystemConfig *sc = supplementary::SystemConfig::getInstance();
    velocityReduction = (*sc)["KickHelper"]->tryGet<double>(1.0, section.c_str(), "ReducePerMS", NULL);

    shared_ptr<vector<string>> kickSections = (*sc)["KickHelper"]->getSections(section.c_str(), "Points", NULL);
    kickCurvePower.resize(kickSections->size());
    kickCurveDistance.resize(kickSections->size());
    int i = 0;
    for (string subsection : *kickSections)
    {
        kickCurvePower[i] = (*sc)["KickHelper"]->get<double>(section.c_str(), "Points", subsection.c_str(), "power", NULL);
        kickCurveDistance[i] = (*sc)["KickHelper"]->get<double>(section.c_str(), "Points", subsection.c_str(), "distance", NULL);
        i++;
    }
}

KickCurve::~KickCurve()
{
    // TODO Auto-generated destructor stub
}

int KickCurve::getKickPower(double dist, double velocity)
{
    int i = 0;
    while (i < kickCurveDistance.size() && kickCurveDistance[i] < dist)
    {
        i++;
    }
    if (i == kickCurveDistance.size() && dist > kickCurveDistance[i - 1])
        return (int)kickCurvePower[i - 1];

    if (i == 0)
        return (int)kickCurvePower[0];

    double dd = kickCurveDistance[i] - kickCurveDistance[i - 1];
    double da = dist - kickCurveDistance[i - 1];
    double dp = kickCurvePower[i] - kickCurvePower[i - 1];

    return (int)round((kickCurvePower[i - 1] + da / dd * dp) * min(1.0, pow(velocityReduction, velocity / 1000.0)));
}

} /* namespace msl */
