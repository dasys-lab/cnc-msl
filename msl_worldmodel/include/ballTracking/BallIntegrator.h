/*
 * $Id: BallIntegrator.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef BallIntegrator_H
#define BallIntegrator_H

#include "ballTracking/ObjectContainer.h"
#include "ballTracking/TrackingTypes.h"
#include <stdlib.h>
#include <vector>

class BallIntegrator
{

  public:
    BallIntegrator();
    ~BallIntegrator();
    void decreaseDirtyPointCertainty();
    void integratePoint(ObservedPoint p_, double threshold);
    ObservedPoint getPoint();
    ObjectContainer *getContainer();

    static BallIntegrator *getInstance();

    void setRefPosition(Position pos);
    Position getRefPosition();

  private:
    void init();
    void cleanup();

    static BallIntegrator *instance_;

    std::vector<PointHypothesis> points;
    std::vector<ObjectContainer *> containers;

    ObservedPoint currPoint;
    ObjectContainer *currContainer;

    Position refPos;

    double increaseCertainty(double certainty);
    double decreaseCertainty(double certainty);
};

#endif
