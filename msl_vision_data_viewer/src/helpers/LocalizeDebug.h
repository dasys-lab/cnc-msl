/*
 * $Id: LocalizeDebug.h 1874 2007-03-02 20:35:47Z rreichle $
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
#ifndef LocalizeDebug_H
#define LocalizeDebug_H

#include <vector>

using namespace std;
//#include "LineDistanceHelper.h"

struct Particle
{

    double posx;
    double posy;
    double heading;
    double weight;
};

struct LinePoint
{

    double x;
    double y;
};

class LocalizeDebug
{

  public:
    LocalizeDebug();
    ~LocalizeDebug();

    vector<pair<double, double>> drawFieldForParticle(Particle particle, int number = 0);

  private:
    double Lines[17][2][2];

    void init();
    void cleanup();
};

#endif
