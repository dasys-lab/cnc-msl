/*
 * $Id: Types.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef Types_H
#define Types_H

#include <stdint.h>
#include <vector>

struct Particle
{

    double posx;
    double posy;
    double heading;
    double weight;
};

struct Position
{

    double x;
    double y;
    double heading;
};

struct WeightedPosition
{

    double x;
    double y;
    double heading;
    double weight;
};

struct Point
{

    double x;
    double y;
};

struct ObservedPoint
{

    double x;
    double y;
    double z;
    double angleHori;
    double angleVerti;
    unsigned long long timestamp;
    double confidence;
    bool valid;
};

struct CorrectedOdometry
{
    double posX;
    double posY;
    double posAngle;
    double posCertainty;
    double motAngle;
    double motTranslation;
    double motRotation;
    unsigned long long timestamp;
    double certainty;
};

struct PointHypothesis
{

    ObservedPoint point;
    double certainty;
    bool dirty;
};

struct Velocity
{

    double vx;
    double vy;
};

struct RobotVelocity
{

    double vx;
    double vy;
    double w;
};

struct Goal
{

    Point leftPost;
    Point rightPost;
    double weight;
};

struct FreeArea
{

    double angle1;
    double angle2;
};

struct MovingObject
{

    Point point;
    Velocity velocity;
};

struct ZEstimate
{
    double z;
    double vz;
};

struct MovingRobot
{

    Position position;
    RobotVelocity velocity;
};

struct Point3D
{

    double x;
    double y;
    double z;
    double angleHori;
    double angleVerti;
};

typedef struct
{
    int x;
    int y;
    int minRadius;
    int maxRadius;

    int err;
    int balls;
    int xballsum;
    int yballsum;
    int sizeSum;
} ballCluster;

typedef struct
{
    short left;
    short right;
    short top;
    short bottom;

    short midX;
    short midY;
} ROIData;

struct Holder
{
    double start;
    double end;
};

struct ROI
{

    int top;
    int bottom;
    int left;
    int right;
    double confidence;
};

struct BlobBounds
{

    int left;
    int right;
    int top;
    int bottom;
    int count;
    double minDistance;
    int minX;
    int minY;
};

struct SharedBall
{
    double x;
    double y;
    double evidence;
    double confidence;
    unsigned long long timestamp;
};

struct Circle
{
    int16_t x;
    int16_t y;
    uint16_t radius;
};

struct ImageSize
{
    uint16_t height;
    uint16_t width;
};

class KinectCluster
{
  public:
    int clusterID;

    std::vector<int> pixels;
    int north, south, west, east;
    int height, width;
    int invHeight, invWidth;
    int center;

    Point3D position;
    double confidence;

    inline void addPixel(int p)
    {
        pixels.push_back(p);
    }
};

#endif
